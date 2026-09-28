# Module 2 — Quản lý đồ thị và chuỗi bundle: Hướng dẫn triển khai

Tài liệu này mô tả cách Module 2 (`02_graph_bundle_management`) được triển
khai trong repository `MARS_Modular`, luồng dữ liệu với Module 1 và Module 3,
cách build/test, và mức đáp ứng các hạng mục WBS MD2.1–MD2.3.

Nội dung được đối chiếu với source code trên `main`,
[`IMPLEMENTATION_PLAN.md`](IMPLEMENTATION_PLAN.md) và WBS của dự án. Tên kiểu,
hàm và file được giữ nguyên để có thể tra ngược vào code.

---

## 1. Tổng quan Module 2

### 1.1 Bài toán

Module 2 trả lời câu hỏi của WBS 2.0:

> Robot nhớ vị trí đã đi thế nào và lấy chuỗi chùm tia nào khi kẹt?

Từ kết quả hình học của mỗi lần quan sát, Module 2:

- lưu và cập nhật trạng thái các open point qua nhiều lần quan sát;
- xếp hạng open point theo công thức của paper;
- tích lũy visibility graph từ các quan hệ nhìn thấy có bằng chứng;
- tìm skeleton route đến một open point lịch sử;
- lưu bundle của từng observation và trích bundle sequence theo chiều route;
- chuẩn bị gates cho Module 3 khi hình học tạo được corridor hợp lệ.

### 1.2 Input / Output

| Loại | Nội dung | Kiểu chính |
|---|---|---|
| Input | ID duy nhất của lần quan sát | `ObservationId` |
| Input | Pose robot hiện tại | `mars::common::Pose2D` |
| Input | Goal dùng để ranking | `mars::common::Point2D` |
| Input | Kết quả hình học từ Module 1 | `PerceptionResult` |
| Output | Open point đang active đã xếp hạng | `std::vector<RankedOpenPoint>` |
| Output | Đồ thị nhìn thấy tích lũy | `VisibilityGraph` |
| Output | Skeleton route tới open point được chọn | `GraphPath` |
| Output | Chuỗi bundle theo đúng chiều route | `BundleSequence` |
| Output | Gates hợp lệ hoặc lỗi có kiểu | `GatePreparationResult` |

Một lần cập nhật được đóng gói bằng `GraphBundleUpdate`. Kết quả chẩn đoán của
lần cập nhật nằm trong `UpdateSummary`.

### 1.3 Vị trí trong pipeline

```text
Map / obstacle geometry
        |
        v
Module 1: perceive(...)
        |  PerceptionResult
        v
Module 2: GraphBundleManager
        |  ranking + graph + bundle sequence + gates
        v
Module 3: navigation decision + DAP/funnel
        |
        v
Module 4: simulation, trace và ROS/RViz adapter
```

Module 1 tạo `NeighborSight`, `ClosedSight`, `OpenSight` và `OpenPoint`.
Module 2 dùng các dữ liệu đó để tạo memory, graph và bundle. Module 3 quyết
định trạng thái điều hướng và tạo đường đi cuối cùng.

---

## 2. Cấu trúc thư mục

```text
02_graph_bundle_management/
├── CMakeLists.txt
├── README.md
├── IMPLEMENTATION_GUIDE.md
├── IMPLEMENTATION_PLAN.md
├── include/mars/graph_bundle_management/
│   ├── types.hpp
│   ├── graph_bundle_manager.hpp
│   ├── ranking.hpp
│   ├── open_point_memory.hpp
│   ├── visibility_graph.hpp
│   ├── graph_search.hpp
│   ├── shared_sight_connector.hpp
│   ├── bundle_builder.hpp
│   ├── bundle_history.hpp
│   ├── sequence_extractor.hpp
│   └── gate_preparation.hpp
├── ranking_memory/
│   ├── ranking.cpp
│   └── open_point_memory.cpp
├── visibility_graph/
│   ├── visibility_graph.cpp
│   ├── graph_search.cpp
│   └── shared_sight_connector.cpp
├── bundle_sequences/
│   ├── bundle_builder.cpp
│   ├── bundle_history.cpp
│   ├── sequence_extractor.cpp
│   └── gate_preparation.cpp
├── pipeline/graph_bundle_manager.cpp
├── examples/synthetic_update.cpp
└── tests/test_graph_bundle_management.cpp
```

Ba nhóm source tương ứng trực tiếp với MD2.1, MD2.2 và MD2.3.
`pipeline/graph_bundle_manager.cpp` là lớp orchestration nối các nhóm này.

---

## 3. Các kiểu dữ liệu chính

Các kiểu public nằm trong
`include/mars/graph_bundle_management/types.hpp`.

| Nhóm | Kiểu | Vai trò |
|---|---|---|
| Memory | `OpenPointRecord` | Bản ghi bền vững của một open point đã merge |
| Memory | `OpenPointStatus` | `Active`, `Selected`, `Reached`, `InactiveExplored`, `Invalid` |
| Ranking | `RankedOpenPoint` | Điểm nhìn theo pose/goal hiện tại cùng score |
| Graph | `VisibilityNode`, `VisibilityEdge` | Nút/cạnh và bằng chứng tạo cạnh |
| Graph | `GraphPath` | Danh sách node theo thứ tự đi và tổng cost |
| Bundle | `Bundle` | Fan đoạn thẳng từ concurrent point tới các visible vertex |
| Sequence | `BundleSequence` | Các bundle theo chiều skeleton route |
| Gate | `Gate`, `GatePreparationResult` | Portal hợp lệ hoặc nguyên nhân thất bại |
| Pipeline | `GraphBundleUpdate` | Input đầy đủ cho một lần `update` |
| Pipeline | `RoutePreparationResult` | Skeleton, sequence và kết quả chuẩn bị gate |

`GraphBundleConfig` chứa các policy quan trọng: trọng số ranking, tolerance,
bán kính merge/visited, chính sách search và cờ `gate_contract_enabled`.

---

## 4. Luồng xử lý tổng thể

### 4.1 Cập nhật một observation

`GraphBundleManager::update(...)` thực hiện luồng sau:

```text
GraphBundleUpdate
  -> validate input và quan hệ pose/perception
  -> build bundle của observation hiện tại
  -> ingest/merge open points vào memory
  -> đánh dấu các điểm đã reached
  -> thêm center, open-point nodes và evidence edges vào graph
  -> nối các observation center nếu hai neighbor sight chứng minh nhìn thấy nhau
  -> lưu bundle và neighbor sight
  -> rank lại toàn bộ active open points
  -> trả UpdateSummary
```

Input sai ném `std::invalid_argument`. Các kiểm tra hình học chính được chạy
trước khi memory và graph bị thay đổi.

### 4.2 Chuẩn bị route

`GraphBundleManager::prepare_route(target)` tách khỏi `update`:

```text
OpenPointId được chọn
  -> kiểm tra target còn dùng được
  -> tìm graph node của target
  -> BFS hoặc Dijkstra từ center hiện tại
  -> extract bundle sequence theo chiều route
  -> prepare gates nếu gate contract được bật
  -> trả RoutePreparationResult
```

Nếu thiếu graph path, bundle hoặc hình học gate, API trả failure reason cụ thể.
Code không tự tạo đường thẳng hoặc gate giả để che một bước thất bại.

---

## 5. MD2.1 — Ranking và bộ nhớ open point

### 5.1 Yêu cầu WBS

WBS yêu cầu:

- lọc điểm đã đi hoặc không còn active;
- xếp hạng theo `alpha / distance + beta / angle` so với goal;
- công thức khớp paper và test được không cần robot.

### 5.2 Memory và lifecycle

`OpenPointMemory::ingest(...)` gộp hai điểm khi khoảng cách của chúng không
vượt `open_point_merge_radius`. Điểm đã gộp giữ nguyên `OpenPointId`, đồng
thời cập nhật observation mới nhất và metadata nguồn.

Lifecycle được quản lý riêng với ranking:

- `Active`: được phép xuất hiện trong danh sách candidate;
- `Selected`: đã được chọn;
- `Reached`: robot đã tới gần trong `visited_radius`;
- `InactiveExplored`: vùng tương ứng đã được đánh dấu explored;
- `Invalid`: record không còn hợp lệ.

Chỉ record `Active` được đưa vào ranking. Các hàm `mark_selected`,
`mark_reached`, `mark_explored`, `reactivate` và `invalidate` thay đổi trạng
thái tường minh.

### 5.3 Công thức ranking

Theo Section 4.3 của paper:

```text
score(p, Ct) = alpha / distance(p, goal)
             + beta  / angle(p, Ct, goal)
```

`angle` là góc nhỏ nhất trong `[0, pi]` giữa hướng từ pose hiện tại tới open
point và hướng tới goal. Mẫu số gần 0 với trọng số khác 0 cho
`+infinity`. Khi score bằng nhau, code phá hòa theo khoảng cách, thứ tự phát
hiện và ID để kết quả ổn định.

Ranking được tính lại sau mỗi `update`; score không được lưu cố định trong
memory vì nó phụ thuộc pose và goal hiện tại.

### 5.4 File và kiểm thử

- `ranking_memory/ranking.cpp`: tính distance, angle, score và thứ tự ranking.
- `ranking_memory/open_point_memory.cpp`: merge, ID và lifecycle.
- `test_ranking()`: kiểm tra công thức, mẫu số 0 và config không hợp lệ.
- `test_memory()`: kiểm tra merge ID, metadata và chuyển trạng thái.

---

## 6. MD2.2 — Visibility Graph

### 6.1 Yêu cầu WBS

WBS yêu cầu nối các pose đã đi và open point thành graph, sau đó chạy được
BFS/skeleton offline.

### 6.2 Cách tạo graph

Graph tích lũy ba loại node:

- `ObservationCenter`: center của một lần quan sát;
- `OpenPoint`: candidate do Module 1 tạo;
- `SharedSightAnchor`: anchor dùng cho quan hệ shared sight khi cần.

Mỗi observation tạo cạnh center–open point có bằng chứng từ neighbor sight
cùng observation. Hai center thuộc hai observation chỉ được nối khi hai
neighbor sight cùng xác nhận đầu kia nằm trong vùng nhìn và đoạn nối không
cắt visible boundary. Khoảng cách gần nhau một mình không đủ để tạo cạnh.

Cạnh là vô hướng, lưu `VisibilityEvidence`, các observation hỗ trợ và cost
Euclid.

### 6.3 Tìm skeleton route

Module cung cấp hai policy với ý nghĩa khác nhau:

| Policy | Hàm | Tối ưu |
|---|---|---|
| `BreadthFirst` | `breadth_first_path(...)` | Số cạnh ít nhất |
| `ShortestCost` | `shortest_cost_path(...)` | Tổng cost hình học nhỏ nhất |

Manager mặc định dùng `ShortestCost`; caller có thể truyền policy tường minh
cho `prepare_route`.

### 6.4 File và kiểm thử

- `visibility_graph/visibility_graph.cpp`: thêm/merge node và cạnh.
- `visibility_graph/graph_search.cpp`: BFS và Dijkstra.
- `visibility_graph/shared_sight_connector.cpp`: kiểm tra bằng chứng nối hai sight.
- `test_search_policies()`: chứng minh BFS và Dijkstra có thể chọn hai route khác nhau.
- `test_shared_sight_proof()`: kiểm tra range và boundary-intersection proof.

---

## 7. MD2.3 — Sequences of Bundles và gates

### 7.1 Yêu cầu WBS

Khi cần lui, WBS yêu cầu dùng hình học sight đã lưu để tạo bundle sequence và
gates cho funnel. Tiêu chí nghiệm thu là `prepare_gates`/bundle sequence gọi
được độc lập node.

### 7.2 Xây dựng và lưu bundle

`build_bundle(...)` dùng `neighbor_sight.visible_boundaries` của Module 1:

1. lấy hai endpoint của mỗi visible boundary không suy biến;
2. loại vertex trùng theo tolerance;
3. tính góc cực quanh observation center;
4. sắp vertex ổn định theo góc;
5. tạo các segment từ concurrent point tới từng vertex;
6. lưu các obstacle edge đã nhìn thấy để chuẩn bị corridor/gate.

`BundleHistory` lưu bundle theo cả `observation_id` và `center_node_id`.

### 7.3 Trích bundle sequence

`extract_bundle_sequence(...)` duyệt skeleton path, bỏ các graph node không
sở hữu bundle, tra bundle tương ứng và giữ đúng chiều hành trình:

- `CurrentToTarget`: từ vị trí hiện tại tới target;
- `TargetToCurrent`: chiều quay lui từ target về hiện tại.

Thiếu bundle hoặc gặp bundle suy biến trả `SequenceFailureReason`; code không
đảo sequence dựa trên ID observation.

### 7.4 Chuẩn bị gate

Khi `gate_contract_enabled=true`, `prepare_gates(...)` dựng corridor C*,
triangulation và lấy các portal của sleeve làm gate. Gate phải hữu hạn, có độ
rộng dương, giữ thứ tự và orientation nhất quán. Hình học không hỗ trợ trả
`UnsupportedGeometry` hoặc `InvalidGate`.

`RoutePreparationResult::success` chỉ xác nhận skeleton và bundle sequence đã
chuẩn bị được. Cần kiểm tra riêng `gate_preparation.success`. Khi contract
đang tắt, kết quả gate là `ContractNotEnabled`.

### 7.5 File và kiểm thử

- `bundle_sequences/bundle_builder.cpp`: dựng bundle từ visible boundaries.
- `bundle_sequences/bundle_history.cpp`: lưu và tra bundle.
- `bundle_sequences/sequence_extractor.cpp`: giữ đúng thứ tự route.
- `bundle_sequences/gate_preparation.cpp`: corridor, triangulation, sleeve và gates.
- `test_bundle_and_direction()`: kiểm tra bundle và chiều sequence.
- `test_visible_boundary_gates()`: kiểm tra gate sinh từ hình học boundary.

---

## 8. Public API và ví dụ sử dụng

Điểm vào chính là `GraphBundleManager`:

```cpp
#include "mars/graph_bundle_management/graph_bundle_manager.hpp"
#include "mars/perception_geometry/perception_pipeline.hpp"

namespace m2 = mars::graph_bundle_management;

m2::GraphBundleManager manager;
m2::GraphBundleUpdate update;
update.observation_id = 1;
update.current_pose = {{0.0, 0.0}, 0.0};
update.goal = {3.0, 0.0};
update.perception = mars::perception_geometry::perceive(
    update.current_pose.position, 2.0, obstacles);

const auto summary = manager.update(update);
const auto ranked = manager.ranked_open_points();

if (!ranked.empty()) {
  const auto route = manager.prepare_route(ranked.front().id);
  if (route.success) {
    // route.skeleton_path và route.bundle_sequence đã sẵn sàng.
    // Kiểm tra route.gate_preparation.success riêng.
  }
}
```

Giữ cùng một instance `manager` qua các observation. Tạo manager mới ở mỗi
frame sẽ làm mất open-point memory, graph và bundle history.

Ví dụ hoàn chỉnh nằm tại `examples/synthetic_update.cpp`.

---

## 9. Build và kiểm thử

Từ thư mục gốc `MARS_Modular/`:

```sh
cmake -S . -B .build/m2 \
  -DBUILD_TESTING=OFF \
  -DMARS_BUILD_TESTS=ON \
  -DMARS_DEMO_BUILD_TESTS=OFF

cmake --build .build/m2 --target \
  mars_graph_bundle_management_tests \
  mars_graph_bundle_management_example

.build/m2/02_graph_bundle_management/mars_graph_bundle_management_tests
.build/m2/02_graph_bundle_management/mars_graph_bundle_management_example
```

Trên Windows/Visual Studio, thêm `--config Debug` và chạy executable trong
`02_graph_bundle_management/Debug/`.

Test executable hiện bao phủ:

- ranking và memory lifecycle;
- BFS so với Dijkstra;
- shared-sight connection proof;
- bundle construction và sequence direction;
- manager pipeline và typed gate failure;
- gate từ visible boundaries;
- input lỗi không làm thay đổi state;
- hợp đồng thật từ Module 1 sang Module 2.

### 9.1 Trạng thái kiểm thử hiện tại

Với `main` tại commit `76e8595`, build M2 và example chạy được trên
Windows/MinGW 14.2. Example in:

```text
Observation 1 produced 1 active open point(s).
```

Test executable hiện dừng ở ca gate với:

```text
Test failure: C* produced no sleeve edge
```

Do đó chưa thể ghi nhận toàn bộ MD2.3 là pass trên cấu hình này. MD2.1 và
MD2.2 có implementation và test case tương ứng; nhánh gate của MD2.3 cần sửa
hoặc xác minh thêm trước khi nghiệm thu toàn bộ M2.

---

## 10. Chạy trong mô phỏng Mode 1

Module 4 đã nối M1 → M2 → M3 trong executable `polygon_explore_sim`:

```sh
cmake -S . -B .build/mars \
  -DBUILD_TESTING=OFF \
  -DMARS_DEMO_BUILD_TESTS=OFF

cmake --build .build/mars --target polygon_explore_sim
.build/mars/04_demo_simulation/polygon_explore_sim --map hard_alley
```

Có thể thay `hard_alley` bằng `geogebra`. Chương trình chạy headless, in trace
và tạo `mode1_<map>.svg`. Mô phỏng thành công cho thấy pipeline tích hợp chạy
được trên map đó, nhưng không thay thế test riêng của gate.

---

## 11. Đối chiếu WBS

| WBS | Yêu cầu / tiêu chí nghiệm thu | Implementation chính | Test evidence | Trạng thái theo code hiện tại |
|---|---|---|---|---|
| 2.0 | Bộ nhớ không gian và chuỗi bundle; ranking khớp paper, BFS skeleton offline | `GraphBundleManager`, ba nhóm MD2.x | `test_manager_pipeline`, contract M1→M2 | Đã tích hợp; chưa nghiệm thu toàn bộ vì gate test còn fail |
| MD2.1 | Lọc điểm không active; ranking `alpha/d + beta/angle`; test không cần robot | `ranking.cpp`, `open_point_memory.cpp` | `test_ranking`, `test_memory` | Đã triển khai và có test |
| MD2.2 | Nối pose/open point thành graph; BFS/skeleton chạy offline | `visibility_graph.cpp`, `graph_search.cpp`, `shared_sight_connector.cpp` | `test_search_policies`, `test_shared_sight_proof` | Đã triển khai và có test |
| MD2.3 | Bundle sequence và gates cho funnel; gọi độc lập node | `bundle_builder.cpp`, `bundle_history.cpp`, `sequence_extractor.cpp`, `gate_preparation.cpp` | `test_bundle_and_direction`, `test_visible_boundary_gates` | Đã triển khai, nhưng gate test hiện chưa pass trên MinGW |

Lưu ý: cột trạng thái trong file WBS gốc chưa được cập nhật theo source. Bảng
trên đánh giá từ code và kết quả chạy hiện tại, không sao chép trạng thái
`Not started` cũ trong workbook.

---

## 12. Phạm vi và hạn chế hiện tại

- Ranking chọn candidate theo heuristic; score cao không tự bảo đảm quỹ đạo
  an toàn cho robot có footprint hữu hạn.
- Cạnh giữa các observation phụ thuộc vào bằng chứng trong neighbor sight đã
  lưu. Không có bằng chứng thì graph có thể bị rời và route trả `NoGraphPath`.
- `prepare_route()` và `return_gates()` phục vụ hai hướng dùng khác nhau:
  route tới target và corridor quay lui qua history.
- Gate contract mặc định tắt. Khi bật, một route hợp lệ vẫn có thể không tạo
  được gate nếu corridor/sleeve không thỏa điều kiện hình học.
- DAP, funnel shortest path và quyết định `EXPLORE`/`ESCAPE` thuộc Module 3.
- Kết quả mô phỏng hiện tại chưa thay thế kiểm thử trên robot thật.

---

## 13. Tóm tắt

Module 2 đã có đầy đủ các khối chính mà WBS mô tả:

- MD2.1: memory, lifecycle và ranking theo công thức paper;
- MD2.2: visibility graph, BFS và shortest-cost search;
- MD2.3: bundle history, route-ordered sequence và gate preparation;
- pipeline: `GraphBundleManager` nối các khối và cung cấp API cho Module 3.

Phần còn cần xử lý trước khi tuyên bố hoàn tất M2 là xác minh/sửa ca gate
`C* produced no sleeve edge` để test MD2.3 pass ổn định trên cấu hình mục tiêu.
