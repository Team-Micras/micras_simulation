<!-- markdownlint-disable -->
<div align="center">

![micras_simulation_blue](https://github.com/Team-Micras/micras_simulation/assets/62271285/655d90d7-ae21-47df-b6ab-64d46ef4a559)

NTF Classic Micromouse simulation environment

</div>

<p align="center">
<a href="https://docs.ros.org/en/humble/index.html"><img alt="ROS Humble" src="https://img.shields.io/badge/ROS_version-humble-informational?style=for-the-badge" height="30"></a>
<a href="http://gazebosim.org/"><img alt="Gazebo Fortress" src="https://img.shields.io/badge/gazebo_version-fortress-important?style=for-the-badge" height="30"></a>
</p>
<p align="center">
<a href="https://cplusplus.com/"><img alt="Made with C++" src="https://img.shields.io/badge/made_with-c%2B%2B-blue?style=for-the-badge&labelColor=ef4041&color=c1282d" height="30"></a>
<a href="https://www.docker.com/"><img alt="Docker" src="https://img.shields.io/badge/docker-container-blue?style=for-the-badge&labelColor=00c5f4&color=0096d6" height="30"></a>
</p>
<!-- markdownlint-restore -->

## 📑 Summary

- [📑 Summary](#-summary)
- [📁 Folder structure](#-folder-structure)
- [🔨 Building](#-building)
- [🚀 Running](#-running)
- [🧪 Testing](#-testing)
- [🐛 Debugging](#-debugging)
- [🐋 Docker](#-docker)
- [💄 Code style](#-code-style)
  - [🎨 Format](#-format)
  - [🚨 Linter](#-linter)
  - [💬 Git commit messages](#-git-commit-messages)
  - [🔀 Git workflow](#-git-workflow)
- [👥 Contributors](#-contributors)

## 📁 Folder structure

- **.vscode** - Visual Studio Code configuration files
- **cmake/** - Functions to include in the main CMake
- **config/** - Target and constants configuration values
- **docker/** - Dockerfiles and scripts to build and run the project
- **gazebo/** - Gazebo world, models and plugin files
- **include/** - Header files for class definitions
- **launch/** - ROS2 launch files
- **MicrasFirmware/** - Micras firmware source code
- **src/** - Source file for HAL and Proxy mocks
- **tests/** - Executable test files mocks

## 🔨 Building

To build the project, you need to run the following command in the current [colcon workspace](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html) root:

```bash
colcon build
```

## 🚀 Running

Before running the project, it is necessary to source the workspace:

```bash
source colcon_workspace/install/setup.bash
```

To run the project, use the following command:

```bash
ros2 launch micras_simulation micras.launch.xml
```

## 🧪 Testing

To run the tests, it is necessary to compile the project with the `COMPILE_TESTS` flag set to `1`:

```bash
colcon build --cmake-args -DCOMPILE_TESTS=1
```

After compiling the project, run the following command to execute the tests, replacing `[test_name]` with the desired test name without the file extension at the end:

```bash
ros2 launch micras_simulation micras.launch.xml name:=[test_name]
```

## 🐛 Debugging

It is possible to debug the project using [`gdb`](https://www.gnu.org/software/gdb/). To do that, first install `gdb-multiarch`, on Ubuntu, just run:

```bash
sudo apt install gdb-multiarch
```

To be able to debug the project, it is necessary run the `colcon build` command with the `BUILD_TYPE` set to `Debug` or `RelWithDebInfo`, for example:

```bash
colcon build --cmake-args -DBUILD_TYPE=Debug
```

## 🐋 Docker

To build the Docker image and run the simulation, use the following command:

```bash
docker compose run sim
```

For building the Docker image for running the tests, use the following command:

```bash
docker compose run test
```

After runnint the command above, a terminal will be opened, and you can run the tests using the following command:

```bash
ros2 launch micras_simulation micras.launch.xml name:=[test_name]
```

## 💄 Code style

### 🎨 Format

The project uses `clang-format` to format files, there is a `.clang-format` with the formatting rules for the project. To install it, on Ubuntu, run the following command on the terminal:

```bash
sudo apt install clang-format
```

In order to format the project, run the following command:

```bash
./format.sh
```

### 🚨 Linter

The project uses a linter in order to follow the best code practices. The linter used is `clang-tidy`, there is a `.clang-tidy` with the linting rules for the project. To install it on Ubuntu, run the following command on the terminal:

```bash
sudo apt install clang-tidy
```

The linting process is done when compiling the project using a special config variable, the `LINTER_MODE` cmake variable. You can enable the linter by running:

```bash
colcon build --cmake-args -DLINTER_MODE=ON
```

To disable the linter while compiling, do as follows:

```bash
colcon build --cmake-args -DLINTER_MODE=OFF
```

It is also possible to lint the project and let the linter fix it using its suggestions:

```bash
colcon build --cmake-args -DLINTER_MODE=FIX
```

### 💬 Git commit messages

- Use the present tense ("Add feature" not "Added feature")
- Use the imperative mood ("Move cursor to..." not "Moves cursor to...")
- It is strongly recommended to start a commit message with a related emoji
  - 📝 `:memo:` for documentation
  - 🐛 `:bug:` for bug issues
  - 🚑 `:ambulance:` for critical fixes
  - 🎨 `:art:` for formatting code
  - ✨ `:sparkles:` for new features

  For more examples, see [this reference](https://gitmoji.carloscuesta.me/).

### 🔀 Git workflow

The project workflow is based on [Git Flow](https://nvie.com/posts/a-successful-git-branching-model/).

## 👥 Contributors

Thanks goes to these wonderful people ([emoji key](https://allcontributors.org/docs/en/emoji-key)):

<!-- ALL-CONTRIBUTORS-LIST:START - Do not remove or modify this section -->
<!-- prettier-ignore-start -->
<!-- markdownlint-disable -->
<table>
  <tr>
    <td align="center"><a href="https://github.com/GabrielCosme"><img src="https://avatars.githubusercontent.com/u/62270066?v=4?s=100" width="100px;" alt="Gabriel Cosme Barbosa"/><br/><sub><b>Gabriel Cosme Barbosa</b></sub></a><br/><a href="https://github.com/Team-Micras/micras_simulation/commits?author=GabrielCosme" title="Code">💻</a> <a href="https://github.com/Team-Micras/micras_simulation/commits?author=GabrielCosme" title="Documentation">📖</a> <a href="#research-GabrielCosme" title="Research">🔬</a> <a href="https://github.com/Team-Micras/micras_simulation/pulls?q=is%3Apr+reviewed-by%3AGabrielCosme" title="Reviewed Pull Requests">👀</a></td>
    <td align="center"><a href="https://github.com/PedroDeSanti"><img src="https://avatars.githubusercontent.com/u/62271285?v=4" width="100px;" alt="Pedro de Santi"/><br/><sub><b>Pedro de Santi</b></sub></a><br/><a href="https://github.com/Team-Micras/micras_simulation/commits?author=PedroDeSanti" title="Code">💻</a> <a href="https://github.com/Team-Micras/micras_simulation/commits?author=PedroDeSanti" title="Documentation">📖</a> <a href="#research-PedroDeSanti" title="Research">🔬</a> <a href="https://github.com/Team-Micras/micras_simulation/pulls?q=is%3Apr+reviewed-by%3APedroDeSanti" title="Reviewed Pull Requests">👀</a></td>
    <td align="center"><a href="https://github.com/Matheus3007"><img src="https://avatars.githubusercontent.com/u/53058455?v=4" width="100px;" alt="Matheus Rezende Pereira"/><br/><sub><b>Matheus Rezende Pereira</b></sub></a><br/><a href="https://github.com/Team-Micras/micras_simulation/commits?author=Matheus3007" title="Code">💻</a> <a href="https://github.com/Team-Micras/micras_simulation/commits?author=Matheus3007" title="Documentation">📖</a> <a href="#research-Matheus3007" title="Research">🔬</a> <a href="https://github.com/Team-Micras/micras_simulation/pulls?q=is%3Apr+reviewed-by%3AMatheus3007" title="Reviewed Pull Requests">👀</a></td>
    <td align="center"><a href="https://github.com/Eduardo-Barreto"><img src="https://avatars.githubusercontent.com/u/34964398?v=4" width="100px;" alt="Eduardo Barreto"/><br/><sub><b>Eduardo Barreto</b></sub></a><br/><a href="https://github.com/Team-Micras/micras_simulation/commits?author=Eduardo-Barreto" title="Code">💻</a> <a href="https://github.com/Team-Micras/micras_simulation/pulls?q=is%3Apr+reviewed-by%3AEduardo-Barreto" title="Reviewed Pull Requests">👀</a></td>
  </tr>
</table>

<!-- markdownlint-restore -->
<!-- prettier-ignore-end -->

<!-- ALL-CONTRIBUTORS-LIST:END -->

This project follows the [all-contributors](https://github.com/all-contributors/all-contributors) specification. Contributions of any kind welcome!
