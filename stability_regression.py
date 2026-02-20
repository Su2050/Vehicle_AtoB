import sys
import statistics
import argparse

sys.path.insert(0, '/Users/sull/Documents/forklift_sim_model')
import main


def main_run():
    parser = argparse.ArgumentParser()
    parser.add_argument('--rounds', type=int, default=20, help='每个case重复次数')
    parser.add_argument('--quick', action='store_true', help='仅跑4个代表场景')
    args = parser.parse_args()

    prims = main.init_primitives()
    cases = [
        (2.8, 0.55, 0.0, True, 'y=+0.55'),
        (2.8, 0.65, 0.0, True, 'y=+0.65'),
        (2.8, 0.75, 0.0, True, 'y=+0.75'),
        (2.8, -0.55, 0.0, True, 'y=-0.55'),
        (2.8, -0.65, 0.0, True, 'y=-0.65'),
        (2.8, -0.75, 0.0, True, 'y=-0.75'),
        (2.8, 0.6, 0.5, True, 'y=+0.6 th=0.5'),
        (2.8, -0.6, -0.5, True, 'y=-0.6 th=-0.5'),
    ]
    if args.quick:
        cases = [
            (2.8, 0.55, 0.0, True, 'y=+0.55'),
            (2.8, 0.75, 0.0, True, 'y=+0.75'),
            (2.8, -0.65, 0.0, True, 'y=-0.65'),
            (2.8, 0.6, 0.5, True, 'y=+0.6 th=0.5'),
        ]
    rounds = max(1, args.rounds)

    all_times = []
    all_ok = 0
    all_total = 0

    print(f'{rounds}轮稳定性回归（中等y+联合偏差）')
    print('-' * 86)
    for x0, y0, th0, no_corridor, label in cases:
        times = []
        oks = 0
        mode_2s = 0
        for _ in range(rounds):
            st = {}
            ok, _, _ = main.plan_path_robust(
                x0, y0, th0, prims,
                no_corridor=no_corridor,
                stats=st
            )
            t = st.get('elapsed_ms', 0.0)
            times.append(t)
            all_times.append(t)
            all_total += 1
            if ok:
                oks += 1
                all_ok += 1
            if st.get('two_stage'):
                mode_2s += 1

        p50 = statistics.median(times)
        p95 = sorted(times)[int(len(times) * 0.95) - 1]
        print(
            f'{label:<16} ok={oks:2d}/{rounds}  '
            f'p50={p50:7.1f}ms  p95={p95:7.1f}ms  '
            f'min={min(times):7.1f}  max={max(times):7.1f}  mode2S={mode_2s:2d}'
        )

    print('-' * 86)
    all_p50 = statistics.median(all_times)
    all_p95 = sorted(all_times)[int(len(all_times) * 0.95) - 1]
    print(f'总体成功率: {all_ok}/{all_total} = {all_ok / all_total * 100:.1f}%')
    print(f'总体时延  : p50={all_p50:.1f}ms  p95={all_p95:.1f}ms  max={max(all_times):.1f}ms')


if __name__ == '__main__':
    main_run()
