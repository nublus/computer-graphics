# computer-graphics
This is a repo recording the experiments conducted during my junior year in Computer Graphics course

## How do I run it
```zsh
cd dandelion
mkdir build
cd build
cmake -S .. -B . -DCMAKE_BUILD_TYPE=Debug
cmake --build . --parallel 8
cmake -S .. -B . -DCMAKE_BUILD_TYPE=Release
cmake --build . --parallel 8
```

More infomation [Dandelion](https://dandelion-docs.readthedocs.io/zh-cn/latest/index.html)
