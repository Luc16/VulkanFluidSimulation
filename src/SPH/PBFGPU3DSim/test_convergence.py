import matplotlib.pyplot as plt
import subprocess


def main():
    res = subprocess.run(["make", "PBFGPU3DSim"], cwd="../../../cmake-build-release", capture_output=True, text=True)
    print(res.stdout)
    res = subprocess.run(["./PBFGPU3DSim"], cwd="../../../cmake-build-release", capture_output=True, text=True)
    data = res.stdout.split("end")

    frames = []
    gauss2 = []
    jacobi = []

    for d in data[0].split("\n"):
        if d:
            (f, g2) = [float(x) for x in d.split("\t")]
            frames.append(f)
            gauss2.append(g2)

    plt.plot(frames, gauss2, label="Gauss-Seidel")

    frames = []
    for d in data[1].split("\n"):
        if d:
            (f, j) = [float(x) for x in d.split("\t")]
            frames.append(f)
            jacobi.append(j)

    plt.plot(frames, jacobi, label="Jacobi")
    plt.legend()
    plt.show()


if __name__ == "__main__":
    main()
