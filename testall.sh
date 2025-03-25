#!/bin/bash

# 检查 build/a1 是否存在
if [ ! -f "build/a1" ]; then
  echo "Error: build/a1 does not exist. Please build the project first."
  exit 1
fi

# 检查 swp 文件夹是否存在
if [ ! -d "swp" ]; then
  echo "Error: swp folder does not exist."
  exit 1
fi

# 遍历 swp 文件夹下的所有 .swp 文件
for swp_file in swp/*.swp; do
  # 检查是否有 .swp 文件
  if [ -f "$swp_file" ]; then
    echo "Processing $swp_file..."
    # 运行 build/a1 并传入当前 .swp 文件
    build/a1 "$swp_file"
    echo "Finished processing $swp_file."
  else
    echo "No .swp files found in swp folder."
  fi
done

echo "All .swp files processed."