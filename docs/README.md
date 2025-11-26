# Misc. Information

Supporting multiple python version is performed via the pyenv utility (https://github.com/pyenv/pyenv)

## pyenv Install

Linux

```shell
curl -fsSL https://pyenv.run | bash
```

macOS

```shell
brew update
brew install pyenv
```

## python3.13

```shell
pyenv install 3.13.9
```

An optimized version of python can also be installed by custom-building the python executable

```shell
apt update
apt install python-build
PYTHON_CONFIGURE_OPTS='--enable-optimizations --with-lto'   python-build -v 3.13.9 $(pyenv root)/versions/3.13.9-opt
```