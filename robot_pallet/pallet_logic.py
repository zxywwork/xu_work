# -*- coding: utf-8 -*-
"""
码垛纯计算逻辑（不依赖 UI / 机器人连接）。

职责：
  - PalletPlanGenerator.generate：根据 P1/P2/P3 参考点 + 行列层参数
    生成码垛目标位姿列表（原 on_generate_pallet_points 的数学部分）；
  - make_approach_depart：由目标位姿构造趋近点 / 离开点位姿。
"""
import math

from .common import build_pose


class PalletPlanGenerator:
    """码垛点生成器（纯计算）。"""

    @staticmethod
    def generate(p1, p2, p3, row, col, layer, height, z_offset,
                 spacing_mode, row_dist=None, col_dist=None):
        """生成码垛目标点位姿列表。

        参数：
          p1/p2/p3 : 参考点 Pose（p2 决定行方向，p3 决定列方向，均以 p1 为基准）
          row/col/layer : 行 / 列 / 层数
          height        : 层高(mm)
          z_offset      : Z 基准偏移(mm)
          spacing_mode  : 0=等分（由 row/col 均分行/列向量）；
                          1=设定间距（row_dist/col_dist 决定步长）
          row_dist/col_dist : 设定间距模式下的行距 / 列距(mm)

        返回：[Pose, ...]，层优先循环（layer → row → col），与原始实现一致。
        """
        row_vec = (p2.x - p1.x, p2.y - p1.y, p2.z - p1.z)
        col_vec = (p3.x - p1.x, p3.y - p1.y, p3.z - p1.z)

        if spacing_mode == 0:
            # 等分模式：把行/列向量平均切分
            if row > 1:
                row_step = (row_vec[0] / (row - 1), row_vec[1] / (row - 1), row_vec[2] / (row - 1))
            else:
                row_step = (0, 0, 0)
            if col > 1:
                col_step = (col_vec[0] / (col - 1), col_vec[1] / (col - 1), col_vec[2] / (col - 1))
            else:
                col_step = (0, 0, 0)
        else:
            # 设定间距模式：按单位向量 * 设定距离
            row_len = math.sqrt(row_vec[0] ** 2 + row_vec[1] ** 2 + row_vec[2] ** 2)
            col_len = math.sqrt(col_vec[0] ** 2 + col_vec[1] ** 2 + col_vec[2] ** 2)
            if row_len < 1e-6 or col_len < 1e-6:
                raise ValueError("P2 或 P3 与 P1 重合，无法计算间距方向")
            row_unit = (row_vec[0] / row_len, row_vec[1] / row_len, row_vec[2] / row_len)
            col_unit = (col_vec[0] / col_len, col_vec[1] / col_len, col_vec[2] / col_len)
            row_step = (row_unit[0] * row_dist, row_unit[1] * row_dist, row_unit[2] * row_dist)
            col_step = (col_unit[0] * col_dist, col_unit[1] * col_dist, col_unit[2] * col_dist)

        points = []
        for k in range(layer):
            z_layer_offset = k * height
            for i in range(row):
                for j in range(col):
                    x = p1.x + i * row_step[0] + j * col_step[0]
                    y = p1.y + i * row_step[1] + j * col_step[1]
                    z_base = p1.z + i * row_step[2] + j * col_step[2] + z_layer_offset
                    z = z_base + z_offset
                    points.append(build_pose(x, y, z, p1.a, p1.b, p1.c))
        return points


def make_approach_depart(target_pose, approach_offset, depart_offset):
    """由目标位姿构造趋近点 / 离开点位姿（仅在 Z 方向加偏移）。

    返回：(approach_pose, depart_pose)
    """
    approach_pose = build_pose(target_pose.x, target_pose.y,
                               target_pose.z + approach_offset,
                               target_pose.a, target_pose.b, target_pose.c)
    depart_pose = build_pose(target_pose.x, target_pose.y,
                             target_pose.z + depart_offset,
                             target_pose.a, target_pose.b, target_pose.c)
    return approach_pose, depart_pose
