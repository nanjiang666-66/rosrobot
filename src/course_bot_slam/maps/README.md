# 保存的 SLAM 地图

建图完成后把以下四个同名前缀文件保存在本目录：

- `course_map.pgm`：占据栅格图像；
- `course_map.yaml`：图像分辨率、原点和阈值；
- `course_map.posegraph`：SLAM Toolbox 序列化位姿图；
- `course_map.data`：与位姿图配套的激光数据。

`PGM/YAML` 适合显示和地图服务器读取；SLAM Toolbox 定位模式需要
`posegraph/data`。不要手工编辑二进制位姿图文件，也不要只复制其中一个文件。

完整建图、保存与验收命令见上一级 [`README.md`](../README.md)。
