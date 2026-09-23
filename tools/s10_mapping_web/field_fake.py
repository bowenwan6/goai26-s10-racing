"""Explicit local-only demonstration adapter. Never selected by production."""
import time

from field_core import binding


class FakeAdapter:
    demo = True

    def __init__(self):
        self.map_name = '1209_01_F-20260912-175844'
        self.invocation = 'demo-invocation'
        self.status_code = 0
        self.start = time.time()-60

    def calibration(self):
        return dict(verified=False, reference_frame=None, version=None, evidence=None,
                    note='演示保持未验证，不能产出真机可导航路线')

    def snapshot(self):
        now = time.time()
        return dict(robot_id='DEMO-NOT-A-ROBOT', boot_id='demo-boot', map_name=self.map_name,
                    board_time=now,
                    map_identity=self.map_name+':demo-sha', invocation=self.invocation,
                    started_at=self.start, mapping_active=False, localization_active=True,
                    status=dict(fresh=True, code=self.status_code, mode='全局'),
                    navigation=dict(fresh=True, idle=True, command=[0,0,0], motion=[0,0,0],
                                    reason='演示：模拟空闲，不代表真实停车', stamp=now, invocation='demo-planner'),
                    pose=dict(frame='map', child_frame='', xyz=[0, 0, 1], quaternion=[0, 0, 0, 1],
                              age=0, stamp_age_s=0, stamp=now, covariance=[0]*36),
                    aligned_cloud=dict(frame='map', age=0, stamp=now,
                                       points=[[0, 1, 0], [1, 1, 0], [2, 1, 1]]),
                    calibration=self.calibration(), demo=True)

    def selfcheck(self, target):
        return dict(passed=False, demo=True, checks=[dict(name='演示环境', state='unknown',
                    detail='模拟数据，不代表真实传感器/地图状态')], target_map=target,
                    binding=binding(self.snapshot()), snapshot=self.snapshot())

    def load_map(self, target, progress):
        progress('演示：模拟切图，不连接机器人')
        self.map_name = target
        self.invocation = 'demo-'+str(time.time_ns())
        return dict(loaded=True, demo=True)

    def sample(self, seconds, progress):
        # Keep actual bounded waiting, so phone-close/reconnect tests are real.
        start=time.monotonic()
        end, rows, last_tick = start+seconds, [], -1
        progress_interval = 1 if seconds <= 5 else 5
        while time.monotonic() < end:
            tick=int(time.monotonic()-start)
            if tick//progress_interval != last_tick:
                progress(f'采样 {tick}/{seconds} 秒（演示计时，不连接机器人）')
                last_tick=tick//progress_interval
            rows.append(self.snapshot()); time.sleep(.1)
        progress('采样结束，正在检查数据与保存结果；请继续等待')
        return rows

    def preview(self, target):
        return dict(frame='map', map_identity=target+':demo-sha', map_name=target,
                    demo=True, points=[[x, y, .1*y] for x in range(-5, 6) for y in range(-5, 6)])

    def record(self, root, kind, seconds, expected, progress):
        progress('演示录制计时；不会产生真实 ROS bag')
        time.sleep(seconds)
        return dict(passed=False, kind=kind, demo=True, reasons=['演示模式没有真实原始数据'], files=[])
