"""Read-only waypoint repeatability metrics, never a navigation authorization."""
import html
import math

from field_core import FieldError, finite_list

REVISIT_SECONDS = 5
REFERENCE_LIMITS = dict(horizontal_m=.10, vertical_m=.10, yaw_deg=5.)


def source_compatible(source, snapshot, calibration_identity):
    """Compare immutable map/reference semantics, NOT localization run identity."""
    old = source.get('binding') or {}
    for key in ('robot_id', 'map_identity'):
        if not old.get(key) or old.get(key) != snapshot.get(key):
            raise FieldError('原航点与当前机器人/地图内容不同，不能复测比较')
    if not source.get('calibration_identity') or source['calibration_identity'] != calibration_identity:
        raise FieldError('原航点的参考点/标定配置缺失或已变化，不能混合比较')
    pose, current = source.get('pose') or {}, snapshot.get('pose') or {}
    if pose.get('passed') is not True or pose.get('frame') != 'map' or current.get('frame') != 'map':
        raise FieldError('原航点不是已通过采样的 map 坐标')
    if 'child_frame' not in pose or pose.get('child_frame') != current.get('child_frame'):
        raise FieldError('定位参考子帧不同或缺失，不能比较同一参考点')
    if not finite_list(pose.get('xyz'), 3) or not finite_list(pose.get('quaternion'), 4):
        raise FieldError('原航点坐标/姿态无效')
    if not .99 <= sum(v*v for v in pose['quaternion']) <= 1.01:
        raise FieldError('原航点姿态无效')
    spread, angle = pose.get('spread_m'), pose.get('angle_rad')
    if (type(spread) not in (int, float) or type(angle) not in (int, float)
            or not math.isfinite(spread) or not math.isfinite(angle)
            or not 0 <= spread <= .10 or not 0 <= angle <= math.radians(5)):
        raise FieldError('原航点采样离散较大或质量信息缺失；请另存可靠基准，不覆盖旧点')


def strict_stationary(quality, samples):
    if not quality.get('passed'):
        raise FieldError('复测采样未通过：'+'；'.join(quality.get('reasons', [])))
    # Same stationary quality rules as saving a WP. Do not impose a hidden
    # stricter 2cm condition only at revisit; report smaller spread as uncertainty.


def compare(source, returned, current_binding, note, ruler_offset_cm=None, alignment_mode='nearby'):
    base = source['pose']
    delta = [b-a for a,b in zip(base['xyz'], returned['xyz'])]
    def yaw(q):
        norm=math.sqrt(sum(v*v for v in q));x,y,z,w=[v/norm for v in q]
        return math.atan2(2*(w*z+x*y),1-2*(y*y+z*z))
    dy = yaw(returned['quaternion'])-yaw(base['quaternion'])
    yaw_deg = math.degrees(math.atan2(math.sin(dy),math.cos(dy)))
    horizontal, vertical = math.hypot(delta[0],delta[1]), delta[2]
    metrics=dict(delta_map_m=delta,horizontal_m=horizontal,vertical_m=vertical,
                 distance_3d_m=math.sqrt(sum(v*v for v in delta)),yaw_deg=yaw_deg)
    within = horizontal <= REFERENCE_LIMITS['horizontal_m'] and abs(vertical) <= REFERENCE_LIMITS['vertical_m']
    cross_run = any(source['binding'].get(k)!=current_binding.get(k) for k in ('boot_id','invocation'))
    calibration=source.get('calibration') or {}
    verified=bool(calibration.get('verified') is True and calibration.get('reference_frame')
                  and calibration['reference_frame']==base.get('child_frame')
                  and calibration.get('version') and calibration.get('evidence'))
    warnings=['这是两次定位坐标的差值，不是纯定位误差，也不是地图绝对精度或导航安全验收。',
              '高度为定位参考点高度差，不是地面高差；站高/姿态变化会影响结果。']
    if alignment_mode=='nearby':warnings.append('附近快速对照：包含真实停靠距离，不能据此给定位精度打分；不要求精确归位。')
    else:warnings.append('尽量对准标记：人工对点残差仍包含在差值中，不假设完全重合。')
    warnings.append('朝向可以不同；朝向差单独记录，不作为位置复测失败条件。未核实参考点偏置时，转身也可能造成坐标变化。')
    precision = max(base['spread_m'], returned['spread_m']) <= .03 and max(base['angle_rad'],returned['angle_rad']) <= math.radians(2)
    if not precision:warnings.append('本次或原点采样有较大离散；已保留数据，请结合散布看差值，不能当成厘米级精度证明。')
    if not verified:warnings.append('参考点/外参尚未核实：仅作诊断对照，不能认定实际机身定位精度。')
    if cross_run:warnings.append('跨开机或重定位会话；已要求当前会话重新定位验收，原航点未改写。')
    if ruler_offset_cm is not None:warnings.append('尺量值为操作者填写的对点残差，未独立核验，不会用于自动校正坐标。')
    return dict(measurement_valid=True,comparison_version=2,comparison_scope='physical_marker_coordinate_difference',
                metrics=metrics,reference_limits=REFERENCE_LIMITS,within_reference=within,
                heading_within_reference=abs(yaw_deg)<=REFERENCE_LIMITS['yaw_deg'],same_heading_required=False,
                alignment_mode=alignment_mode,exact_alignment_claimed=False,position_error_isolated=False,
                sampling_precision_sufficient=precision,
                sampling_spread_m=dict(original=base['spread_m'],returned=returned['spread_m']),
                reference_verified=verified,cross_localization_run=cross_run,
                physical_alignment_confirmed=alignment_mode=='careful',human_marker_return_confirmed=True,
                reference_note=note,ruler_offset_cm=ruler_offset_cm,
                reference_pose=base,return_pose=returned,source_binding=source['binding'],binding=current_binding,
                warnings=warnings,absolute_accuracy_verified=False,navigation_ready=False,
                source_waypoint_unchanged=True)


def comparison_svg(result):
    """Self-contained metric XY diagram, not an environment or ground-truth map."""
    dx,dy,_=result['metrics']['delta_map_m'];extent=max(.12,abs(dx)*1.25,abs(dy)*1.25)
    scale=200/extent;x=260+dx*scale;y=260-dy*scale
    name=html.escape(('演示 · ' if result.get('demo') else '')+result['source_name'])
    return (f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 520 560" role="img">'
            '<rect width="520" height="560" fill="#f5f8f5"/>'
            f'<text x="20" y="28" font-size="16">{name} 返回复测 · XY偏差放大图</text>'
            '<path d="M40 260H480M260 50V470" stroke="#aaa"/>'
            f'<circle cx="260" cy="260" r="{.1*scale:.2f}" fill="none" stroke="#87a" stroke-dasharray="5 5"/>'
            f'<path d="M260 260L{x:.2f} {y:.2f}" stroke="#bc5a18" stroke-width="2"/>'
            '<circle cx="260" cy="260" r="6" fill="#315dcc"/>'
            f'<circle cx="{x:.2f}" cy="{y:.2f}" r="5" fill="#bc5a18"/>'
            f'<text x="20" y="500" font-size="14">蓝=原坐标，橙=返回坐标；轴范围 ±{extent*100:.1f} cm</text>'
            '<text x="20" y="523" font-size="14">虚线=10 cm参考线；不是地形图或绝对精度证明</text>'
            '<text x="20" y="546" font-size="14">X向右 / Y向上；地图坐标轴，不是机身左右</text></svg>')
