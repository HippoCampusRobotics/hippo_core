# Hippo-Core
[Our Website](https://hippocampusrobotics.github.io/)

This repository contains packages/libraries used on onboard/offboard devices and are required in every setup.

## Check for formatting errors
```sh
find ./ -iname '*.hpp' -o -iname '*.cpp' | xargs clang-format --dry-run --style=file:./.clang-format
```

## Apply formatting to all files
```sh
find ./ -iname '*.hpp' -o -iname '*.cpp' | xargs clang-format -i --style=file:./.clang-format
```
