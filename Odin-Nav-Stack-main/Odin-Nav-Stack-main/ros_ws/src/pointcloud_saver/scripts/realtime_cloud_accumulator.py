#!/usr/bin/env python3
"""
Realtime Point Cloud Accumulator for Odin1
============================================
实时点云累积节点 —— 启动即自动累积 Odin1 点云，实时在 RViz 中显示。

Features:
  - 启动后自动开始累积，无需手动 start/stop
  - 实时发布累积后的完整点云到 /accumulated_cloud
  - 支持在线体素下采样，控制内存占用
  - 提供暂停/继续、清空、保存服务
  - TF 坐标变换自动处理

Usage:
  roslaunch pointcloud_saver realtime_cloud_accumulator.launch

Services:
  - /realtime_cloud_accumulator/pause       (std_srvs/Trigger) 暂停累积
  - /realtime_cloud_accumulator/resume      (std_srvs/Trigger) 继续累积
  - /realtime_cloud_accumulator/clear_map   (std_srvs/Trigger) 清空已累积点云
  - /realtime_cloud_accumulator/save_map    (pointcloud_saver/SaveMap) 保存为 PCD

Author: AI Assistant for Odin1
"""

import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from pointcloud_saver.srv import SaveMap, SaveMapResponse
from std_srvs.srv import Trigger, TriggerResponse
import tf2_ros
import struct
import threading
from datetime import datetime
import os


class RealtimeCloudAccumulator:
    def __init__(self):
        rospy.init_node("realtime_cloud_accumulator", anonymous=False)

        # ===================== Parameters =====================
        self.cloud_topic = rospy.get_param("~cloud_topic", "/odin1/cloud_slam")
        self.target_frame = rospy.get_param("~target_frame", "map")
        self.save_dir = rospy.get_param("~save_dir", os.path.expanduser("~/maps"))
        self.max_points = rospy.get_param("~max_points", 10000000)
        self.voxel_size = rospy.get_param("~voxel_size", 0.05)
        self.save_rgb = rospy.get_param("~save_rgb", True)
        self.pcd_binary = rospy.get_param("~pcd_binary", True)
        self.max_stat_points = rospy.get_param("~max_stat_points", 200000)
        self.publish_rate = rospy.get_param("~publish_rate", 2.0)
        self.auto_start = rospy.get_param("~auto_start", True)
        self.max_frames_per_sec = rospy.get_param("~max_frames_per_sec", 10.0)

        # ===================== State =====================
        self.is_accumulating = self.auto_start
        self.frame_count = 0
        self.voxel_map = {}  # key: (vx,vy,vz) -> [sum_x, sum_y, sum_z, sum_r, sum_g, sum_b, count]
        self.map_lock = threading.Lock()
        self.last_frame_time = rospy.Time(0)

        # ===================== TF =====================
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # ===================== IO =====================
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)
            rospy.loginfo(f"[Accumulator] 创建保存目录: {self.save_dir}")

        # 订阅点云
        self.cloud_sub = rospy.Subscriber(
            self.cloud_topic, PointCloud2, self.cloud_callback, queue_size=10
        )

        # 发布累积点云
        self.accumulated_pub = rospy.Publisher(
            "/accumulated_cloud", PointCloud2, queue_size=1
        )

        # 定时发布器（兼顾性能与实时性）
        self.pub_timer = rospy.Timer(
            rospy.Duration(1.0 / self.publish_rate), self.publish_accumulated_cloud
        )

        # ===================== Services =====================
        self.pause_srv = rospy.Service(
            "~pause", Trigger, self.pause_callback
        )
        self.resume_srv = rospy.Service(
            "~resume", Trigger, self.resume_callback
        )
        self.clear_srv = rospy.Service(
            "~clear_map", Trigger, self.clear_map_callback
        )
        self.save_srv = rospy.Service(
            "~save_map", SaveMap, self.save_map_callback
        )

        # ===================== Log =====================
        rospy.loginfo("=" * 60)
        rospy.loginfo("[RealtimeCloudAccumulator] 实时点云累积节点已启动")
        rospy.loginfo(f"  订阅话题: {self.cloud_topic}")
        rospy.loginfo(f"  目标坐标系: {self.target_frame}")
        rospy.loginfo(f"  发布话题: /accumulated_cloud")
        rospy.loginfo(f"  体素下采样: {self.voxel_size}m")
        rospy.loginfo(f"  自动开始累积: {self.auto_start}")
        rospy.loginfo(f"  发布频率: {self.publish_rate} Hz")
        rospy.loginfo(f"  最大点数限制: {self.max_points}")
        rospy.loginfo("=" * 60)
        rospy.loginfo("  服务列表:")
        rospy.loginfo("    ~/pause      - 暂停累积")
        rospy.loginfo("    ~/resume     - 继续累积")
        rospy.loginfo("    ~/clear_map  - 清空已累积点云")
        rospy.loginfo("    ~/save_map   - 保存为 PCD 文件")
        rospy.loginfo("=" * 60)

    # ============================================================
    #  Callbacks
    # ============================================================

    def cloud_callback(self, msg):
        """点云回调：接收并累积到体素地图"""
        if not self.is_accumulating:
            return

        # 帧率限制
        now = rospy.Time.now()
        dt = (now - self.last_frame_time).to_sec()
        if dt < (1.0 / self.max_frames_per_sec):
            return
        self.last_frame_time = now

        # 检查点数上限
        with self.map_lock:
            if len(self.voxel_map) >= self.max_points:
                if self.frame_count % 100 == 0:
                    rospy.logwarn(f"[Accumulator] 已达到最大点数限制 {self.max_points}")
                return

        # 获取 TF 变换
        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                msg.header.frame_id,
                rospy.Time(0),
                rospy.Duration(1.0),
            )
        except Exception as e:
            if self.frame_count % 50 == 0:
                rospy.logwarn(f"[Accumulator] TF 查询失败: {e}")
            return

        tx = transform.transform.translation.x
        ty = transform.transform.translation.y
        tz = transform.transform.translation.z
        qx = transform.transform.rotation.x
        qy = transform.transform.rotation.y
        qz = transform.transform.rotation.z
        qw = transform.transform.rotation.w

        R = self.quaternion_to_rotation_matrix(qx, qy, qz, qw)
        t = np.array([tx, ty, tz])

        # 处理点云数据
        new_points = 0
        for point in pc2.read_points(msg, skip_nans=True):
            x, y, z = point[:3]
            p_local = np.array([x, y, z])
            p_map = R @ p_local + t

            # 颜色
            r = g = b = 255
            if self.save_rgb and len(point) >= 4:
                rgb = point[3]
                if isinstance(rgb, float):
                    rgb_int = struct.unpack("I", struct.pack("f", rgb))[0]
                    r = (rgb_int >> 16) & 0xFF
                    g = (rgb_int >> 8) & 0xFF
                    b = rgb_int & 0xFF

            # 体素索引
            vs = self.voxel_size
            vx = int(np.floor(p_map[0] / vs))
            vy = int(np.floor(p_map[1] / vs))
            vz = int(np.floor(p_map[2] / vs))
            key = (vx, vy, vz)

            with self.map_lock:
                if key not in self.voxel_map:
                    if self.save_rgb:
                        self.voxel_map[key] = [
                            float(p_map[0]), float(p_map[1]), float(p_map[2]),
                            float(r), float(g), float(b), 1.0
                        ]
                    else:
                        self.voxel_map[key] = [
                            float(p_map[0]), float(p_map[1]), float(p_map[2]), 1.0
                        ]
                else:
                    acc = self.voxel_map[key]
                    acc[0] += float(p_map[0])
                    acc[1] += float(p_map[1])
                    acc[2] += float(p_map[2])
                    if self.save_rgb:
                        acc[3] += float(r)
                        acc[4] += float(g)
                        acc[5] += float(b)
                        acc[6] += 1.0
                    else:
                        acc[3] += 1.0
            new_points += 1

        self.frame_count += 1

        if self.frame_count % 30 == 0:
            with self.map_lock:
                count = len(self.voxel_map)
            rospy.loginfo(f"[Accumulator] 已累积 {count} 个体素，来自 {self.frame_count} 帧")

    def publish_accumulated_cloud(self, event=None):
        """定时发布累积点云到 /accumulated_cloud"""
        with self.map_lock:
            values = list(self.voxel_map.values())

        if not values:
            return

        # 计算体素质心
        has_rgb = self.save_rgb
        centroids = []
        for v in values:
            if has_rgb:
                count = max(1.0, v[6])
                cx = v[0] / count
                cy = v[1] / count
                cz = v[2] / count
                cr = int(v[3] / count)
                cg = int(v[4] / count)
                cb = int(v[5] / count)
                centroids.append((cx, cy, cz, cr, cg, cb))
            else:
                count = max(1.0, v[3])
                cx = v[0] / count
                cy = v[1] / count
                cz = v[2] / count
                centroids.append((cx, cy, cz))

        # 如果点数太多，均匀采样（发布性能考虑）
        if len(centroids) > 100000:
            step = max(1, len(centroids) // 100000)
            centroids = centroids[::step]

        try:
            header = rospy.Header()
            header.stamp = rospy.Time.now()
            header.frame_id = self.target_frame

            if has_rgb and len(centroids) > 0 and len(centroids[0]) >= 6:
                # 创建带 RGB 的 PointCloud2
                fields = [
                    pc2.PointField('x', 0, pc2.PointField.FLOAT32, 1),
                    pc2.PointField('y', 4, pc2.PointField.FLOAT32, 1),
                    pc2.PointField('z', 8, pc2.PointField.FLOAT32, 1),
                    pc2.PointField('rgb', 12, pc2.PointField.UINT32, 1),
                ]
                points = []
                for x, y, z, r, g, b in centroids:
                    rgb_int = (int(r) << 16) | (int(g) << 8) | int(b)
                    points.append((float(x), float(y), float(z), rgb_int))
                cloud_msg = pc2.create_cloud(header, fields, points)
            else:
                points = [(float(p[0]), float(p[1]), float(p[2])) for p in centroids]
                cloud_msg = pc2.create_cloud_xyz32(header, points)

            self.accumulated_pub.publish(cloud_msg)

        except Exception as e:
            rospy.logerr(f"[Accumulator] 发布点云失败: {e}")

    # ============================================================
    #  Services
    # ============================================================

    def pause_callback(self, req):
        self.is_accumulating = False
        rospy.loginfo("[Accumulator] 已暂停累积")
        return TriggerResponse(success=True, message="已暂停累积")

    def resume_callback(self, req):
        self.is_accumulating = True
        rospy.loginfo("[Accumulator] 已恢复累积")
        return TriggerResponse(success=True, message="已恢复累积")

    def clear_map_callback(self, req):
        with self.map_lock:
            old_count = len(self.voxel_map)
            self.voxel_map.clear()
        self.frame_count = 0
        rospy.loginfo(f"[Accumulator] 已清空 {old_count} 个体素")
        return TriggerResponse(success=True, message=f"已清空 {old_count} 个体素")

    def save_map_callback(self, req):
        """保存累积点云为 PCD"""
        try:
            rospy.loginfo("[Accumulator] 开始保存地图...")

            with self.map_lock:
                values = list(self.voxel_map.values())

            if not values:
                return SaveMapResponse(
                    success=False, message="没有累积的点云数据",
                    saved_path="", total_points=0, filtered_points=0
                )

            # 提取质心
            raw_points = []
            for v in values:
                if self.save_rgb:
                    count = max(1.0, v[6])
                    raw_points.append((
                        v[0] / count, v[1] / count, v[2] / count,
                        int(v[3] / count), int(v[4] / count), int(v[5] / count)
                    ))
                else:
                    count = max(1.0, v[3])
                    raw_points.append((v[0] / count, v[1] / count, v[2] / count))

            total_points = len(raw_points)
            rospy.loginfo(f"[Accumulator] 原始点数: {total_points}")

            # 体素滤波
            voxel_size = req.voxel_size if req.voxel_size > 0 else self.voxel_size
            filtered = self.voxel_filter_fast(raw_points, voxel_size)

            # 统计滤波
            if req.apply_statistical_filter and len(filtered) > 100:
                try:
                    filtered = self.statistical_outlier_filter(filtered)
                except Exception as e:
                    rospy.logwarn(f"[Accumulator] 统计滤波失败: {e}")

            # 生成文件名
            if req.filename:
                filename = req.filename if req.filename.endswith(".pcd") else req.filename + ".pcd"
            else:
                filename = f"accumulated_map_{datetime.now().strftime('%Y%m%d_%H%M%S')}.pcd"

            filepath = os.path.join(self.save_dir, filename)
            self.save_pcd(filtered, filepath)

            rospy.loginfo(f"[Accumulator] 保存成功: {filepath}")
            rospy.loginfo(f"  原始点数: {total_points} -> 滤波后: {len(filtered)}")

            return SaveMapResponse(
                success=True, message="地图保存成功",
                saved_path=filepath, total_points=total_points,
                filtered_points=len(filtered)
            )

        except Exception as e:
            rospy.logerr(f"[Accumulator] 保存失败: {e}")
            import traceback
            traceback.print_exc()
            return SaveMapResponse(
                success=False, message=f"保存失败: {str(e)}",
                saved_path="", total_points=0, filtered_points=0
            )

    # ============================================================
    #  Helpers
    # ============================================================

    def quaternion_to_rotation_matrix(self, qx, qy, qz, qw):
        R = np.array([
            [1 - 2*(qy**2 + qz**2), 2*(qx*qy - qz*qw), 2*(qx*qz + qy*qw)],
            [2*(qx*qy + qz*qw), 1 - 2*(qx**2 + qz**2), 2*(qy*qz - qx*qw)],
            [2*(qx*qz - qy*qw), 2*(qy*qz + qx*qw), 1 - 2*(qx**2 + qy**2)],
        ])
        return R

    def voxel_filter_fast(self, points, voxel_size):
        if voxel_size <= 0 or not points:
            return points

        has_rgb = len(points[0]) >= 6
        if has_rgb:
            xyz = np.array([(p[0], p[1], p[2]) for p in points], dtype=np.float32)
            rgb = np.array([(p[3], p[4], p[5]) for p in points], dtype=np.float32)
        else:
            xyz = np.array([(p[0], p[1], p[2]) for p in points], dtype=np.float32)
            rgb = None

        if xyz.shape[0] == 0:
            return []

        vox = np.floor(xyz / float(voxel_size)).astype(np.int64)
        keys, inv, counts = np.unique(vox, axis=0, return_inverse=True, return_counts=True)

        cx = np.bincount(inv, weights=xyz[:, 0]) / counts
        cy = np.bincount(inv, weights=xyz[:, 1]) / counts
        cz = np.bincount(inv, weights=xyz[:, 2]) / counts
        centroids = np.stack([cx, cy, cz], axis=1).astype(np.float32)

        if has_rgb and rgb is not None:
            cr = np.bincount(inv, weights=rgb[:, 0]) / counts
            cg = np.bincount(inv, weights=rgb[:, 1]) / counts
            cb = np.bincount(inv, weights=rgb[:, 2]) / counts
            colors = np.stack([cr, cg, cb], axis=1).astype(np.float32)
            out = np.concatenate([centroids, colors], axis=1)
        else:
            out = centroids

        rospy.loginfo(f"[Accumulator] 体素滤波: {xyz.shape[0]} -> {out.shape[0]} 点")
        result = []
        for p in out:
            if has_rgb:
                result.append((float(p[0]), float(p[1]), float(p[2]), int(p[3]), int(p[4]), int(p[5])))
            else:
                result.append((float(p[0]), float(p[1]), float(p[2])))
        return result

    def statistical_outlier_filter(self, points, k=50, std_ratio=1.0):
        if len(points) < k:
            return points
        if len(points) > self.max_stat_points:
            rospy.logwarn(f"[Accumulator] 跳过统计滤波（点数 {len(points)} > {self.max_stat_points}）")
            return points

        from scipy.spatial import cKDTree
        xyz = np.array([(p[0], p[1], p[2]) for p in points])
        tree = cKDTree(xyz)

        mean_distances = []
        for i, point in enumerate(xyz):
            distances, _ = tree.query(point, k=k+1)
            mean_dist = np.mean(distances[1:])
            mean_distances.append(mean_dist)

        mean_distances = np.array(mean_distances)
        global_mean = np.mean(mean_distances)
        global_std = np.std(mean_distances)
        threshold = global_mean + std_ratio * global_std

        filtered = [points[i] for i in range(len(points)) if mean_distances[i] <= threshold]
        rospy.loginfo(f"[Accumulator] 统计滤波: {len(points)} -> {len(filtered)} 点")
        return filtered

    def save_pcd(self, points, filename):
        rospy.loginfo(f"[Accumulator] 保存 PCD: {filename}")
        has_rgb = self.save_rgb and points and len(points[0]) >= 6

        if not self.pcd_binary:
            # ASCII
            with open(filename, "w") as f:
                f.write("# .PCD v0.7 - Point Cloud Data file format\n")
                f.write("VERSION 0.7\n")
                if has_rgb:
                    f.write("FIELDS x y z rgb\nSIZE 4 4 4 4\nTYPE F F F U\nCOUNT 1 1 1 1\n")
                else:
                    f.write("FIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n")
                f.write(f"WIDTH {len(points)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n")
                f.write(f"POINTS {len(points)}\nDATA ascii\n")
                if has_rgb:
                    for p in points:
                        rgb_int = (int(p[3]) << 16) | (int(p[4]) << 8) | int(p[5])
                        f.write(f"{p[0]:.6f} {p[1]:.6f} {p[2]:.6f} {rgb_int}\n")
                else:
                    for p in points:
                        f.write(f"{p[0]:.6f} {p[1]:.6f} {p[2]:.6f}\n")
            rospy.loginfo(f"[Accumulator] ASCII 保存完成: {len(points)} 点")
            return

        # Binary
        N = len(points)
        with open(filename, "wb") as f:
            header = "# .PCD v0.7 - Point Cloud Data file format\n"
            header += "VERSION 0.7\n"
            if has_rgb:
                header += "FIELDS x y z rgb\nSIZE 4 4 4 4\nTYPE F F F U\nCOUNT 1 1 1 1\n"
            else:
                header += "FIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
            header += f"WIDTH {N}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n"
            header += f"POINTS {N}\nDATA binary\n"
            f.write(header.encode("ascii"))

            pack_xyz = struct.Struct("<fff").pack
            pack_rgb = struct.Struct("<I").pack
            if has_rgb:
                for p in points:
                    f.write(pack_xyz(float(p[0]), float(p[1]), float(p[2])))
                    rgb_int = (int(p[3]) << 16) | (int(p[4]) << 8) | int(p[5])
                    f.write(pack_rgb(rgb_int))
            else:
                for p in points:
                    f.write(pack_xyz(float(p[0]), float(p[1]), float(p[2])))
        rospy.loginfo(f"[Accumulator] Binary 保存完成: {len(points)} 点")


def main():
    try:
        acc = RealtimeCloudAccumulator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
