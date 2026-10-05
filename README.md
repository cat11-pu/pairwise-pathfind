# pathfind

网格寻路与视野内核：网格 A* 寻路（含斜走与带代价地形）与切比雪夫视野，纯标准库 Python 3。

## 内容

- pathfind/core.py：Grid（墙体、带代价地形）、find_path、path_cost、compute_fov
- tests/test_core.py：unittest 用例

Grid(width, height, walls=(), terrain=None)：walls 是不可通行的格子集合，
terrain 把格子映射为正的代价倍率，进入该格时按倍率计价。

## 跑测试

在项目根目录执行：

    python3 -m unittest discover -s tests -v
