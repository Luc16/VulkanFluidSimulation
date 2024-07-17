import matplotlib.pyplot as plt
import subprocess
import matplotlib.ticker as mtick


def main():
    plt.gca().yaxis.set_major_formatter(mtick.PercentFormatter(xmax=1.0))

    res = subprocess.run(["make", "PBFGPU3DSim"], cwd="../../../cmake-build-release", capture_output=True, text=True)
    print(res.stdout)
    res = subprocess.run(["./PBFGPU3DSim"], cwd="../../../cmake-build-release", capture_output=True, text=True)
    data = res.stdout

    frames = []
    plot_data = [[]]
    avg_times = []

    for d in data.split("\n"):
        if d:
            sd = d.split("\t")
            if sd[0] == "end":
                plot_data.append([])
                avg_times.append(float(sd[1]))
            else:
                (f, g2) = [float(x) for x in sd]
                if len(plot_data) == 1:
                    frames.append(f)
                plot_data[-1].append(g2)

    converge_frames = []
    for idx, d in enumerate(plot_data[:-1]):
        last_20 = d[0:50]
        first_to_converge = -1
        for i, v in enumerate(d[50:]):
            converged = True
            c = 0
            for j in last_20:
                c = max(c, abs(v - j) / v)
                if abs(v - j) / v > 5e-3:
                    converged = False
                    first_to_converge = -1
                    break
            # print(f"i: {i} c: {c}")
            last_20.pop(0)
            last_20.append(v)
            if converged:
                if first_to_converge == -1:
                    first_to_converge = i + 50
                # print(f'{c=} {c < 1e-30} {v} {abs(v - last_20[0])/v}')
                # print(last_20)
                # print(f"Method {idx} Converged at {i+50}")
        converge_frames.append(first_to_converge)

    print(
        f"4 & {(avg_times[1] * 1000):.2f} & {converge_frames[1] if converge_frames[1] != -1 else '-'} & {(avg_times[0] * 1000):.2f} & {converge_frames[0] if converge_frames[0] != -1 else '-'} \\\\")

    for i in range(len(avg_times)):
        if i == 0:
            plt.plot(frames, plot_data[i], label=f"Red-Black")
        elif i == 1:
            plt.plot(frames, plot_data[i], label=f"Jacobi")

    plt.xlabel("Frames")
    plt.ylabel("Mean Density Relative to Rest Density")
    plt.legend()
    plt.show()


if __name__ == "__main__":
    main()
