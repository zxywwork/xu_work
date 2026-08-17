# -*- coding: utf-8 -*-
"""
视觉标定纯逻辑模块（不直接操作 UI 控件）。

职责：
  - point_to_dict / point_from_dict : x5.Point <-> dict 序列化；
  - build_config : 由参数 dict 构造 x5v.Vision 通信配置；
  - build_calib_process : 由参数 dict 构造 x5v.VisionCalibProcess 标定工艺；
  - save_calib / load_calib : 标定结果 JSON 文件的保存与读取。

UI（MainWindow）负责：采集界面参数（_collect_calib_params）、回填界面、
弹窗提示与调用机器人写入（apply_vision_config / vision_set_calib_par）。
"""
import json
import os

import xapi.api as x5
import xapi.api.vision as x5v


def point_to_dict(point):
    """x5.Point -> dict（可 JSON 序列化）。"""
    if point is None:
        return None
    p = point.pose
    cfg = list(point.cfg) if hasattr(point, 'cfg') else [0, 0, 0, 1]
    return {
        'x': p.x, 'y': p.y, 'z': p.z,
        'a': p.a, 'b': p.b, 'c': p.c,
        'uf': point.uf, 'tf': point.tf, 'cfg': cfg,
    }


def point_from_dict(d):
    """dict -> x5.Point。"""
    if not d:
        return None
    pose = x5.Pose(d['x'], d['y'], d['z'], d['a'], d['b'], d['c'], 0, 0, 0)
    return x5.Point(pose=pose, uf=d.get('uf', 0), tf=d.get('tf', 0),
                    cfg=tuple(d.get('cfg', [0, 0, 0, 1])))


def build_config(params):
    """由标定参数 dict 构造 x5v.Vision 通信配置。"""
    return x5v.Vision(
        ip=str(params['ip']).encode(),
        port=int(params['port']),
        communication_protocol=int(params['protocol']),
        auto_start=False,
        data_format=int(params['data_format']),
        invert_c_value=bool(params['invert_c']),
        camera_mount_type=int(params['mount_type']),
        uf_index=int(params['uf_index']),
        calibration_object=int(params['calib_object']),
        trigger_type=int(params['trigger_type']),
        trigger_do=255,
        trigger_command=str(params['trigger_cmd']).encode()
    )


def build_calib_process(params):
    """由标定参数 dict 构造 x5v.VisionCalibProcess（动态 J2 工艺）。"""
    calib_process = x5v.VisionCalibProcess()
    calib_process.calibration_type = int(params['calib_type'])
    calib_process.point_count = int(params['point_count'])
    calib_process.is_workplane_parallel = True
    calib_process.calib_data.auto_dynamic_not_end = x5v._CalibrationDataUnion._AutoDynamicNotEndData(
        tf_index=int(params['tf_index']),
        pixel_shift=[int(params['pixel_u']), int(params['pixel_v'])],
        error=0.0,
        base_point=point_from_dict(params['base_point']),
        mark_distance=int(params['mark_distance']),
        step_size=int(params['step_size'])
    )
    return calib_process


def save_calib(file_path, vision_idx, params, transformation):
    """把某个视觉工艺的标定参数 + 变换矩阵合并写入 JSON 文件。

    返回写入后的文件路径。
    """
    data = {}
    if os.path.exists(file_path):
        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                data = json.load(f)
        except Exception:
            data = {}
    params = dict(params)
    params['transformation'] = [float(x) for x in transformation]
    data[str(vision_idx)] = params
    with open(file_path, 'w', encoding='utf-8') as f:
        json.dump(data, f, ensure_ascii=False, indent=2)
    return file_path


def load_calib(file_path):
    """读取标定 JSON 文件，返回 {str(vision_idx): params}。"""
    with open(file_path, 'r', encoding='utf-8') as f:
        return json.load(f)
