# Module 1 — Adapter Hình học Nhận thức Cảm biến: Hướng dẫn triển khai

Tài liệu này mô tả cách Module 1 (`01_perception_geometry`) được triển khai thực tế trong
repository `MARS_Modular`, quan hệ của nó với paper tham chiếu và với WBS, cách build/test
độc lập, và cách nó được dùng trong toàn bộ Mode 1.

Toàn bộ nội dung dưới đây được đối chiếu trực tiếp với source code hiện tại. Các định danh
(tên file, tên hàm, tên kiểu, lệnh CMake, ký hiệu toán) giữ nguyên dạng tiếng Anh.

---

## 1. Tổng quan Module 1

### 1.1 Bài toán

Module 1 trả lời câu hỏi của WBS 1.0:

> Robot nhìn gì trong tầm nhìn `r`, đâu là tường, đâu là hướng đi?

Cụ thể, từ một tâm robot `C_t`, một bán kính tầm nhìn `r` và một bản đồ chướng ngại vật
dạng hình học, module tính ra:

- biên chướng ngại vật *thực sự nhìn thấy được* (đã xử lý che khuất),
- các **cung góc bị chặn** (đóng) và **cung góc trống** (mở) trên vòng tròn tầm nhìn,
- một **điểm đại diện** cho mỗi cung trống.

Module thuần hình học: **không ROS, không TF, không driver, không visualization, không goal,
không ranking, không graph, không planning, không motor control**. (xem `01_perception_geometry/README.md`).

### 1.2 Input / Output

| Loại | Nội dung | Kiểu |
|---|---|---|
| Input | Tâm robot `C_t` | `Point2D` |
| Input | Bán kính tầm nhìn `r` | `double` |
| Input | Chướng ngại vật (wall mỏng hoặc polygon đặc) | `std::vector<Polygon2D>` |
| Output | Biên nhìn thấy được | `NeighborSight` |
| Output | Các cung bị chặn | `std::vector<ClosedSight>` |
| Output | Các cung trống | `std::vector<OpenSight>` |
| Output | Điểm đại diện cho cung trống | `std::vector<OpenPoint>` |
| Output | Gói kết quả tổng hợp | `PerceptionResult` |

### 1.3 Vì sao Module 1 độc lập ROS

Toàn bộ logic nằm trong một thư viện C++17 thuần (`mars::perception_geometry`), chỉ phụ thuộc
`common/cpp/include/mars/common/geometry_types.hpp` và chuẩn C++. Hàm public duy nhất là
hàm thuần `perceive(center, radius, obstacles)` — không có state toàn cục, không I/O, không
phụ thuộc topic/message/time. Nhờ vậy có thể test bằng bản đồ giả độc lập, đúng tiêu chí
nghiệm thu "Hàm thuần, test bằng map giả độc lập".

### 1.4 Vị trí trong pipeline toàn hệ thống

```
Bản đồ / chướng ngại vật (Polygon2D)
        │
        ▼
┌──────────────────────────────┐
│ Module 1: perception_geometry│   perceive(center, radius, obstacles)
│  NeighborSight → Closed →    │
│  Open → OpenPoints           │
└──────────────────────────────┘
        │  PerceptionResult
        ▼
┌──────────────────────────────┐
│ Module 2: graph_bundle_mgmt  │   ranking, VisibilityGraph, BundleSequence
└──────────────────────────────┘
        │  Ranked open points / graph / bundles
        ▼
┌──────────────────────────────┐
│ Module 3: navigation_decision│   Algorithm 2 (Explore / BAR / DAP-funnel)
└──────────────────────────────┘
        │  NavigationState / PlannedPath
        ▼
┌──────────────────────────────┐
│ Module 4: demo_simulation    │   fake odom, path follower, ROS/RViz markers, trace
└──────────────────────────────┘
```

Chỉ Module 1 sinh ra `NeighborSight`, `ClosedSight`, `OpenSight`, `OpenPoint`. Các thành phần
`graph`, `bundles`, `funnel` trong trace là output của Module 2/3, **không phải** của Module 1.

---

## 2. Cấu trúc thư mục

```
01_perception_geometry/
├── CMakeLists.txt                       # build 1 library + 1 test executable
├── README.md                            # tài liệu gốc của module
├── include/mars/perception_geometry/    # public API headers
│   ├── types.hpp                        # NeighborSight, ClosedSight, OpenSight, OpenPoint, PerceptionResult
│   ├── geometry.hpp                     # primitives + khai báo helper hình học
│   ├── neighbor_sight.hpp               # compute_neighbor_sight(...)
│   ├── closed_sights.hpp                # compute_closed_sights(...)
│   ├── open_sights.hpp                  # compute_open_sights(...)
│   ├── open_points.hpp                  # compute_open_point(s)(...)
│   └── perception_pipeline.hpp          # perceive(...)
├── geometry/
│   ├── geometry.cpp                     # cài đặt primitives dùng chung
│   ├── test_geometry.cpp                # test primitives
│   └── README.md
├── neighbor_sight/
│   ├── neighbor_sight.cpp               # MD1.1
│   ├── test_neighbor_sight.cpp
│   └── README.md
├── closed_sights/
│   ├── closed_sights.cpp                # MD1.2
│   ├── test_closed_sights.cpp
│   └── README.md
├── open_sights/
│   ├── open_sights.cpp                  # MD1.3 — cung trống
│   ├── open_points.cpp                  # MD1.3 — điểm đại diện
│   ├── test_open_sights.cpp
│   ├── test_open_points.cpp
│   └── README.md
├── pipeline/
│   ├── perception_pipeline.cpp          # perceive(...) nối 4 stage
│   ├── test_pipeline.cpp                # test tích hợp + oracle
│   └── README.md
├── internal/                            # helper private, không export
│   ├── detail.hpp                       # angle, cross, on_segment, crossing, touches
│   └── intervals.hpp                    # linear_union, circular_union, contains
└── tests/
    └── README.md                        # index của test suite
```

| Thư mục | Trách nhiệm |
|---|---|
| `geometry/` | Primitive hình học dùng chung: distance, chuẩn hóa góc, tia/cung/circle, clip. |
| `neighbor_sight/` | MD1.1: clip biên chướng ngại theo đĩa tầm nhìn, xử lý che khuất. |
| `closed_sights/` | MD1.2: chiếu biên nhìn thấy thành cung góc bị chặn. |
| `open_sights/` | MD1.3: phần bù vòng tròn → cung trống, và điểm đại diện. |
| `pipeline/` | Nối 4 stage thành `perceive(...)` + test tích hợp. |
| `internal/` | Helper private (`detail`, `intervals`) chia sẻ giữa các stage. |
| `include/` | API public và khai báo kiểu dữ liệu module. |
| `tests/` | Chỉ mục tài liệu test (test nằm cạnh từng feature). |

---

## 3. Các kiểu dữ liệu chính

Các primitive canonical khai báo trong `common/cpp/include/mars/common/geometry_types.hpp` và
được alias lại trong `include/mars/perception_geometry/types.hpp`:

```cpp
using common::Point2D;
using common::Segment2D;
using common::Polygon2D;
using common::Circle2D;
using common::Pose2D;
using common::AngularInterval;
```

### 3.1 Primitive

| Kiểu | Ý nghĩa | Trường | Khai báo |
|---|---|---|---|
| `Point2D` | Điểm trong mặt phẳng mét | `x`, `y` (double) | `common/.../geometry_types.hpp` |
| `Segment2D` | Đoạn thẳng | `start`, `end` (`Point2D`) | `common/.../geometry_types.hpp` |
| `Polygon2D` | Wall mỏng (2 đỉnh) hoặc polygon đặc (≥3 đỉnh) | `vertices` | `common/.../geometry_types.hpp` |
| `Circle2D` | Đường tròn tầm nhìn | `center`, `radius` | `common/.../geometry_types.hpp` |
| `Pose2D` | Vị trí + hướng | `position`, `yaw` | `common/.../geometry_types.hpp` |
| `AngularInterval` | Cung góc `{start, sweep}` | `start ∈ [0, 2π)`, `sweep ∈ [0, 2π]` | `common/.../geometry_types.hpp` |

Quy ước góc: CCW radian, zero dọc +X. `start ∈ [0, 2π)`, `sweep ∈ [0, 2π]`; sweep = 0 là cung
rỗng, sweep = 2π là toàn vòng. Cung có thể wrap quanh 0 (ví dụ `{350°, 30°}`).

### 3.2 Kiểu output của Module 1

Khai báo trong `01_perception_geometry/include/mars/perception_geometry/types.hpp`.

| Kiểu | Trường | Vai trò | Sinh bởi | Dùng bởi |
|---|---|---|---|---|
| `NeighborSight` | `center`, `radius`, `visible_boundaries` | Biên nhìn thấy sau che khuất (fragment, **không** phải toàn bộ cạnh) | `compute_neighbor_sight` | `compute_closed_sights`, Module 2 |
| `ClosedSight` | `interval`, `visible_boundaries` | Một cung bị chặn + các fragment hỗ trợ | `compute_closed_sights` | `compute_open_sights`, Module 2/4 |
| `OpenSight` | `interval` | Một cung trống | `compute_open_sights` | `compute_open_points`, Module 2/4 |
| `OpenPoint` | `point`, `angle`, `sight_index` (optional) | Điểm đại diện trên vòng tầm nhìn | `compute_open_point(s)` | Module 2/3/4 |
| `PerceptionResult` | `neighbor_sight`, `closed_sights`, `open_sights`, `open_points` | Gói toàn bộ output | `perceive` | Module 2/3/4 |

`NeighborSight.visible_boundaries` lưu ý nghĩa "fragment đã giải che khuất", không phải mọi cạnh
đã clip. `ClosedSight.visible_boundaries` giữ nhiều fragment vì một sector gộp có thể do nhiều
fragment hỗ trợ. `OpenPoint.sight_index` giữ chỉ số nguồn trong vector open sights.

---

## 4. Luồng xử lý tổng thể

Entry point public trong `pipeline/perception_pipeline.cpp`:

```cpp
PerceptionResult perceive(const Point2D& center, double radius,
                          const std::vector<Polygon2D>& obstacles) {
    PerceptionResult result;
    result.neighbor_sight = compute_neighbor_sight(center, radius, obstacles);
    result.closed_sights  = compute_closed_sights(result.neighbor_sight);
    result.open_sights    = compute_open_sights(result.closed_sights);
    result.open_points    = compute_open_points(center, radius, result.open_sights);
    return result;
}
```

Chuỗi gọi thực tế trùng với dạng khái niệm mong đợi:

```
perceive(center, radius, obstacles)
  │
  ├─ compute_neighbor_sight(center, radius, obstacles)   → NeighborSight
  │
  ├─ compute_closed_sights(neighbor_sight)               → vector<ClosedSight>
  │
  ├─ compute_open_sights(closed_sights)                  → vector<OpenSight>
  │
  └─ compute_open_points(center, radius, open_sights)    → vector<OpenPoint>
```

Chú ý: `compute_open_points` nhận `center` và `radius` **gốc**, không đọc lại từ `NeighborSight`;
`compute_open_sights` chỉ nhận `closed_sights` (nó không cần biên gốc).

---

## 5. MD1.1 — Neighbor Sight

File: `neighbor_sight/neighbor_sight.cpp`
Hàm: `compute_neighbor_sight(center, radius, obstacles)`
Khai báo: `include/mars/perception_geometry/neighbor_sight.hpp`

### 5.1 Paper/WBS định nghĩa gì

WBS MD1.1: *"Từ tâm `C_t`, cắt biên vật cản theo vòng tầm nhìn `r`"* — nhận thức biên vật cản
trong tầm nhìn `r`. Paper tham chiếu (Sections 4.1–4.2) mô tả *closed sights* dựa trên biên
chướng ngại **nhìn thấy được** và observer; do đó Module 1 cần một stage xác định biên nào
thực sự nhìn thấy trước khi phân loại góc.

### 5.2 Code thực tế nhận gì

`compute_neighbor_sight` nhận `center`, `radius`, và `std::vector<Polygon2D>`. Trước hết gọi
`validate_circle` (radius phải finite và `> linear_epsilon`). Sau đó dựng cạnh local:

- Mỗi polygon được **tịnh tiến về hệ observer-relative**: `local = p - center`.
- Loại đỉnh trùng liên tiếp trong `linear_epsilon`; loại đỉnh đóng trùng đỉnh đầu.
- Polygon 2 đỉnh → 1 cạnh (wall mỏng). Polygon ≥3 đỉnh → các cạnh nối vòng, có kiểm tra
  topology (đơn giản, không backtrack, có diện tích, observer nằm ngoài).
- Nếu observer nằm trên biên cạnh bất kỳ → `std::invalid_argument`.
- Nếu observer nằm trong polygon đặc → `std::invalid_argument`.

### 5.3 Clip biên theo vòng tầm nhìn

Mỗi cạnh local được đưa qua `clip_segment_to_circle(edge, {{0,0}, radius})`. Kết quả rỗng hoặc
có độ dài ≤ `linear_epsilon` bị loại. Các cạnh đã clip này là ứng viên biên.

### 5.4 Xác định biên nhìn thấy (visibility theo sự kiện góc)

Đây là điểm cốt lõi — **không quét theo độ phân giải cố định**, mà chia vòng tròn tại các
*sự kiện góc* rồi cast một tia giữa mỗi ô.

```
Input:
    center C_t, radius r, obstacle boundaries
Process:
    1. Dịch chuyển cạnh về observer-relative; validate topology/observer
    2. Clip từng cạnh theo circle radius r; bỏ kết quả rỗng
    3. Thu thập critical angles:
         - 0 và 2π (hai đầu trục)
         - góc của điểm đầu/điểm cuối mỗi cạnh đã clip
         - góc giao điểm cặp cạnh (crossing) — xử lý tường giao nhau
       Sắp xếp, khử trùng (unique)
    4. Với mỗi ô [angles[k-1], angles[k]] có bề rộng > angular_epsilon:
         - cast tia tại góc giữa (midpoint) → tìm cạnh gần nhất bị cắt
         - chọn cạnh đó làm biên của ô
         - cắt cạnh đó bằng hai tia biên của ô để thu fragment nhìn thấy
    5. Nối các fragment liền kề cùng một cạnh nguồn (kể cả mối nối tại góc 0)
Output:
    NeighborSight { center, radius, visible_boundaries (global) }
```

Logic đúng vì: giữa hai sự kiện góc liên tiếp, tập cạnh bị cắt và thứ tự độ sâu **không đổi**,
nên một tia nội ô xác định chính xác cạnh gần nhất. Đây là visibility theo interval dựa trên
hình học, không phải quét mẫu. Các offset `±epsilon` quanh góc là không cần thiết vì tia chọn
nằm hẳn trong lòng ô sự kiện.

### 5.5 Xử lý che khuất

- Biên bị che bởi tường gần hơn sẽ không được chọn làm "cạnh gần nhất" trong ô tương ứng → bị
  loại trước khi phân loại.
- Một cạnh bị che một phần có thể sinh **nhiều fragment** (ví dụ tường xa bị tường gần chắn ở
  giữa). Test `NeighborSight.PartialOcclusionPreservesFarFragments` kiểm tra trường hợp này:
  hai tường → 3 fragment, tổng độ dài phần xa = 4.
- Khi hai tường giao nhau, visibility "đổi chủ" tại giao điểm — test
  `NeighborSight.CrossingWallsExchangeVisibility` (tổng độ dài `2√5`).

### 5.6 Trường hợp biên được xử lý

| Trường hợp | Hành vi | Test |
|---|---|---|
| Môi trường rỗng / wall ngoài tầm | `visible_boundaries` rỗng | `EmptyAndOutside` |
| Wall xa bị wall gần che hoàn toàn | chỉ còn fragment tường gần | `WallAndNearFarOcclusion` |
| Che một phần | giữ các fragment tường xa | `PartialOcclusionPreservesFarFragments` |
| Polygon đặc | chỉ thấy mặt trước | `FilledRectangleShowsOnlyFrontFace` |
| Wall dài hơn đường kính | bị clip theo circle | `CircleClipsLongWall` |
| Tangent / radial / degenerate | số đo góc = 0 → không sinh biên | `TangentRadialAndDegenerateHaveZeroAngularMeasure` |
| Wall rất gần / gần tangent | vẫn thấy | `NearWallAndNearTangent` |
| Đỉnh trùng, wall chồng lấn | dedupe, độ dài đúng | `DuplicateVerticesAndOverlappingWalls` |
| Observer trên biên / trong polygon / topology xấu | throw `std::invalid_argument` | `RejectsInvalidObserverAndPolygon` |

### 5.7 Độ phức tạp

Theo `01_perception_geometry/README.md`: **worst-case time O(E³), space O(E²)** với `E` là số
cạnh đã clip. Đây là con số được tài liệu module công bố; bản triển khai ưu tiên tính đúng đắn
có thể kiểm tra (inspectable correctness) hơn là tối ưu cho bản đồ lớn. Tăng tốc cho map lớn
được hoãn lại.

### 5.8 Vì sao test độc lập được không cần ROS

`compute_neighbor_sight` chỉ nhận dữ liệu hình học thuần (`Point2D`, `double`, `vector<Polygon2D>`)
và trả về dữ liệu thuần. Không có ROS type, topic, thời gian hay I/O. Test dựng trực tiếp bản đồ
giả trong bộ nhớ và so sánh output.

### 5.9 Cách tôi triển khai

- **Bài toán → biểu diễn:** thay vì quét tia rời rạc, tôi chọn biểu diễn theo *critical angles*
  (điểm đầu/cuối cạnh đã clip + giao điểm cặp cạnh). Cách này cho kết quả chính xác theo giải
  tích và không phụ thuộc độ phân giải.
- **Chọn thuật toán:** chia vòng tròn thành các ô bởi sự kiện góc; trong mỗi ô, "cạnh gần nhất"
  là bất biến, nên chỉ cần một tia giữa ô. Đây là cách đưa bài toán visibility về dạng
  interval-exact.
- **Edge case:** xử lý riêng seam tại góc 0 (nối fragment bị cắt bởi mốc 0), cạnh degenerate,
  tangent (số đo góc 0), observer trên biên/trong polygon.
- **Testing:** tách test che khuất toàn phần, một phần, giao nhau, chồng lấn, và các trường hợp
  số học gần biên (near-wall, near-tangent).

---

## 6. MD1.2 — Closed Sights

File: `closed_sights/closed_sights.cpp`
Hàm: `compute_closed_sights(const NeighborSight& neighbor)`
Helper: `internal/intervals.hpp` (`linear_union`, `circular_union`, `contains`)

### 6.1 Định nghĩa dùng trong implementation

Một **ClosedSight** là một cung góc liên tục bị chặn, thu được bằng cách chiếu các fragment
biên nhìn thấy lên vòng tròn góc rồi hợp nhất. Đây là bước *phân loại*, **không** chứng minh lại
visibility: header ghi rõ input phải là fragment đã giải che khuất (thường do
`compute_neighbor_sight` sinh ra).

### 6.2 Input → Processing → Output

```
Input:
    NeighborSight (visible_boundaries đã giải che khuất)
Process:
    1. validate_circle cho (center, radius)
    2. Với mỗi visible boundary:
         - validate điểm; kiểm tra cả hai đầu nằm trong circle (radius + linear_epsilon),
           nếu không → throw
         - bỏ fragment có độ dài ≤ linear_epsilon
         - nếu segment đi qua observer → throw
         - dịch về observer-relative
         - tính góc a, b; sweep = normalize(b - a)
           nếu sweep > π thì đảo hướng để lấy cung *nhỏ* (minor arc)
         - bỏ nếu sweep ≤ angular_epsilon
       → thu được các projection AngularInterval + giữ boundaries tương ứng
    3. circular_union(projections): hợp nhất các cung, xử lý wrap và seam
    4. Với mỗi interval hợp nhất:
         - tìm các projection có angular_midpoint nằm trong interval (helper contains)
         - gắn các boundary tương ứng làm visible_boundaries của ClosedSight
Output:
    vector<ClosedSight>, sắp theo start đã chuẩn hóa
```

### 6.3 Quan hệ với biên nhìn thấy & góc

- Mỗi fragment → một cung **nhỏ** (minor arc) trên vòng tròn. Wrap quanh 0 được tách để hợp
  nhất tuyến tính, sau đó mối nối seam vòng tròn được ghép lại (`circular_union`).
- Các interval chồng lấn hoặc chạm nhau (trong `angular_epsilon`) được **merge**.
- ClosedSight gộp có thể được **nhiều fragment hỗ trợ** (không nhất thiết một tam giác đơn).
  Test `OverlappingProjectionsMergeVisibleFragments`: 2 tường → 1 sight, sweep `2·atan2(3,4)`,
  3 boundary.

### 6.4 Biểu diễn hướng bị chặn

Một hướng bị chặn ⇔ góc đó nằm trong `sight.interval`. `detail::contains` dùng
`normalize_angle(angle - interval.start)` và so với `sweep` (có tính `angular_epsilon`), đồng
thời đặc biệt xử lý `sweep == 2π` (toàn vòng).

### 6.5 Trường hợp biên

| Trường hợp | Hành vi | Test |
|---|---|---|
| Wall vắt qua mốc 0 | interval wrap, start `7π/4`, sweep `π/2` | `WallCrossingZero` |
| Hai wall tách biệt | 2 sight, sắp theo start | `SeparatedWallsAreOrdered` |
| Projection chồng | 1 sight gộp, nhiều fragment | `OverlappingProjectionsMergeVisibleFragments` |
| Wall bị che | không xuất hiện trong closed sights | `HiddenWallNotIncluded` |
| Rỗng / degenerate / rất nhỏ | trả rỗng | `EmptyDegenerateAndTiny` |
| Boundary ngoài circle / đi qua observer | throw | `RejectsInvalidBoundaries` |

### 6.6 Cách tôi triển khai

- **Bài toán → biểu diễn:** biên đã nhìn thấy → cung góc; một hướng bị chặn khi nằm trong hợp
  các cung.
- **Chọn thuật toán:** chuẩn hóa mỗi fragment về minor arc, tách wrap, hợp nhất tuyến tính, ghép
  lại seam — dùng chung helper `internal/intervals.hpp` để closed/open sights có cùng quy ước.
- **Edge case:** fragment chỉ chạm observer, fragment dài ≤ epsilon, sweep > π, seam tại 0.
- **Testing:** kiểm tra cả nội dung cung (start/sweep) lẫn số fragment hỗ trợ, đảm bảo thông tin
  provenance không bị mất khi merge.

---

## 7. MD1.3 — Open Sights

File: `open_sights/open_sights.cpp`
Hàm: `compute_open_sights(const std::vector<ClosedSight>& closed)`

### 7.1 Nguyên lý: phần bù vòng tròn

Open sights là **phần bù vòng tròn** của hợp các cung bị chặn:

```
Input:
    vector<ClosedSight> (có thể unsorted / chồng nhau)
Process:
    1. linear_union(tất cả interval đóng) → các span đã sắp xếp, đã merge
    2. Nếu spans rỗng  → trả {0, 2π}  (một vòng tròn mở đầy đủ)
    3. Với mỗi span i:
         end   = (i+1 < size) ? spans[i+1].start : spans[0].start + 2π
         sweep = end - spans[i].end
         nếu sweep > angular_epsilon → thêm { normalize(spans[i].end), sweep }
       (khe seam cuối↔đầu được đo như MỘT khoảng tròn trước khi áp epsilon)
    4. Sắp xếp kết quả theo interval.start
Output:
    vector<OpenSight>
```

### 7.2 Quy ước góc & wrap quanh 0 / 2π

- `start` luôn đưa về `[0, 2π)` bằng `normalize_angle`; `sweep` dương.
- Khe seam (`spans.back().end` → `spans.front().start + 2π`) được tính là một khoảng tròn duy
  nhất. Test `SeamGapUsesCombinedWidth` xác nhận điều này (sweep `2·half` với `half = 0.75·epsilon`).

### 7.3 Trường hợp toàn vòng / rỗng

| Input closed | Output open | Test |
|---|---|---|
| Không có (empty) | 1 sight `{0, 2π}` | `EmptyAndFullCoverage` |
| Phủ toàn vòng | rỗng | `EmptyAndFullCoverage` |
| Sector wrap `{7π/4, π/2}` | `{π/4, 3π/2}` | `WrappingClosedSector` |
| Sector `{π/4, π/2}` | `{3π/4, 3π/2}` | `WrappingOpenSector` |
| Nested/trùng/chồng/unsorted | gộp, phần bù đúng | `UnsortedOverlappingNestedAndDuplicateIntervals` |
| Hai vùng mở | 2 sight | `MultipleOpenRegionsAndTouchingClosedIntervals` |
| Sector/khe ≤ epsilon | bị nuốt (coi như đóng) | `TinyBlockedSectorsAndGaps` |
| Interval không hợp lệ | throw (kể cả đã phủ full) | `InvalidIntervalsEvenAfterFullCoverage` |

### 7.4 Tolerance

`angular_epsilon = 1e-10` (radian): các sector bị chặn nhỏ hơn/≤ ngưỡng bị loại; khe ≤ ngưỡng
bị đóng. Đây là ngưỡng **hợp nhất độ phủ kề nhau** và **ô sector rất nhỏ**, không phải số học
chính xác tùy ý ở mọi thang đo.

### 7.5 Cách tôi triển khai

- **Bài toán → biểu diễn:** đưa về bài toán hợp khoảng tuyến tính (`linear_union`) rồi lấy phần
  bù; điều này tái dùng trực tiếp helper đã kiểm thử ở `internal/intervals.hpp`.
- **Chọn thuật toán:** xử lý wrap bằng cách tách span vắt mốc 0, hợp nhất, rồi đo khe seam như
  một khoảng tròn. Nhờ vậy không cần sentinel đặc biệt cho góc 0.
- **Edge case:** empty → full circle; full coverage → rỗng; khe seam; interval unsorted/trùng.
- **Testing:** kiểm tra riêng wrap đóng, wrap mở, nested, touching, và sai số epsilon.

---

## 8. MD1.3 — Open Points

File: `open_sights/open_points.cpp`
Hàm: `compute_open_point(center, radius, sight, sight_index)` và
`compute_open_points(center, radius, sights)`

### 8.1 Vì sao cần điểm đại diện

Mỗi OpenSight là một *cung* (tập hợp hướng). Downstream (Module 2 ranking, navigation) cần một
**điểm 2D cụ thể** trên cung để tham chiếu/xếp hạng. Module 1 chỉ cung cấp điểm hình học đại
diện, **không xếp hạng, không chọn goal, không clearance/navigation policy**.

### 8.2 Cách tính

```
Input:
    center, radius, OpenSight {interval}, optional sight_index
Process:
    theta = angular_midpoint(interval)      // = normalize(start + sweep/2)
    point = point_on_circle({center, radius}, theta)
Output:
    OpenPoint { point, angle = theta, sight_index }
```

- Điểm nằm trên **trung điểm góc** của cung trống.
- Toạ độ tính bằng `center + radius·(cos θ, sin θ)` (qua `point_on_circle`).
- `compute_open_points` **giữ nguyên thứ tự** vector open sights và gán `sight_index = i` cho
  từng điểm.

### 8.3 Ví dụ toán đơn giản

Với `center = (1, 2)`, `radius = 3`, `sight = {0, π}`:

```
theta = normalize(0 + π/2) = π/2
point = (1 + 3·cos(π/2), 2 + 3·sin(π/2)) = (1, 5)
```

Trùng với test `OpenPoints.SingleSector`. Với full circle `{0, 2π}`: `theta = π`,
`point = (center.x − radius, center.y)`; test `FullCircleAndDeterminism` cho `center=(3,4)`,
`radius=2` → `(1, 4)`, và kết quả lặp lại deterministic.

### 8.4 Lưu ý

- Interval rỗng (`sweep = 0`) không có midpoint → `angular_midpoint` throw `std::invalid_argument`.
- `sight_index` là `std::optional`: API vector luôn gán, API đơn có thể để trống khi caller tự gọi.
- Một điểm cho mỗi sight; **không** chia nhỏ theo goal (khác với hướng mở rộng của paper).

### 8.5 Cách tôi triển khai

- **Bài toán → biểu diễn:** chọn midpoint góc vì nó là đại diện "giữa vùng an toàn", ổn định và
  deterministic, đồng thời không thiên vị hướng.
- **Chọn thuật toán:** tái dùng `angular_midpoint` và `point_on_circle` từ `geometry.cpp` để
  giữ một nguồn quy ước góc duy nhất.
- **Edge case:** interval rỗng (throw), full circle (điểm cố định tại hướng π), radius không hợp lệ.
- **Testing:** midpoint, gán index, determinism, input rỗng/không hợp lệ.

---

## 9. Geometry helpers

File: `geometry/geometry.cpp` (public), `internal/detail.hpp` (private).

| Helper | Ý nghĩa | Vì sao tồn tại |
|---|---|---|
| `distance(a, b)` | khoảng cách Euclid (hypot long double) | so sánh độ dài fragment, kiểm tra degeneracy, oracle |
| `normalize_angle(a)` | đưa góc về `[0, 2π)` qua fmod | một quy ước góc duy nhất cho toàn module |
| `angular_midpoint(i)` | `normalize(start + sweep/2)` | điểm đại diện của cung; từ chối cung rỗng |
| `point_on_circle(c, angle)` | `center + r·(cos, sin)` | biến (center, radius, angle) → toạ độ 2D |
| `ray_segment_intersection(o, angle, s)` | giao tia↔đoạn gần nhất, kể cả endpoint/collinear | chọn cạnh gần nhất trong mỗi ô sự kiện |
| `segment_circle_intersections(s, c)` | các giao điểm đoạn↔đường tròn, sắp theo start→end | tìm sự kiện góc ở biên đĩa; xử lý tangent |
| `clip_segment_to_circle(s, c)` | phần đoạn nằm trong đĩa | cắt biên chướng ngại theo tầm nhìn r |
| `detail::angle(p)` | `atan2` chuẩn hóa | chuyển điểm → góc sự kiện |
| `detail::cross(...)` | tích chéo long double | predicate song song/collinear |
| `detail::on_segment(p, s)` | điểm nằm trên đoạn (trong epsilon) | kiểm tra observer trên biên, đỉnh trùng |
| `detail::crossing(a, b)` | giao duy nhất hai đoạn | sinh sự kiện giao tường |
| `detail::touches(a, b)` | crossing hoặc chạm đầu mút | kiểm tra polygon đơn (simple) |
| `detail::linear_union` / `circular_union` / `contains` | hợp khoảng & kiểm tra chứa góc | lõi của closed/open sights |

Điểm đáng chú ý về độ chính xác: tính giao dùng trung gian `long double`; ngưỡng song song của
tia dùng `64 × machine epsilon` (vì hướng tia đến từ lượng giác double, kể cả `sin(pi)`);
predicate giao đoạn dùng machine precision của long double. Biến `parallel_epsilon =
64 * std::numeric_limits<double>::epsilon()` được khai báo rõ trong `geometry.cpp:11`.

---

## 10. Ví dụ luồng dữ liệu

Lấy ví dụ từ test fixture **có sẵn** trong `pipeline/test_pipeline.cpp`
(`TEST(Pipeline, CaseBOneWall)`):

```
Robot:
    C_t = (0, 0)
    r   = 5

Obstacles:
    một thin wall: Polygon2D{{ (2,-2), (2,2) }}     // Polygon2D 2 đỉnh = wall mỏng
```

Luồng:

```
obstacles [wall x=2, y∈[-2,2]]
        │  compute_neighbor_sight
        ▼
visible fragments: 1 đoạn trên x=2 (mặt trước, đã clip theo đĩa r=5)
        │  compute_closed_sights
        ▼
closed intervals: 1 sector {start=7π/4, sweep=π/2}   (từ hướng -45° đến +45°, vắt mốc 0)
        │  compute_open_sights
        ▼
open intervals: 1 sector còn lại            (tổng coverage = π/2 theo test)
        │  compute_open_points
        ▼
open points: điểm tại midpoint của cung mở, angle = π  (test: EXPECT_NEAR(angle, π))
```

Đối chiếu test: `CaseBOneWall` khẳng định `closed_sights.size() == 1`,
`open_sights.size() == 1`, `coverage == π/2`, `open_points[0].angle == π`, và `check_partition`
xác nhận tổng `closed.sweep + open.sweep = 2π`.

Ví dụ bổ sung từ test `ClosedSights.WallCrossingZero`: cùng wall này cho closed interval
`start = 7π/4`, `sweep = π/2` — minh họa việc một wall thẳng đứng ở phía +X tạo cung bị chặn
vắt qua mốc góc 0.

Các test khác dùng cùng fixture tinh thần: `CaseCTwoSeparatedWalls`, `CaseECorridor`,
`CaseFBlindAlleyPerceptionOnly`, `ConcaveFilledUHasAnOpenMouth`.

---

## 11. Kiểm thử

Test dùng GTest, build bằng CMake thường, chạy qua CTest — **hoàn toàn không ROS**. Tất cả test
nằm cạnh feature tương ứng; `tests/README.md` là chỉ mục.

| File | Số test | Nội dung |
|---|---|---|
| `geometry/test_geometry.cpp` | 7 | primitives, quy ước góc, invalid/overflow, tia (hit/miss/collinear/degenerate), giao & clip circle, tangent/near-tangent |
| `neighbor_sight/test_neighbor_sight.cpp` | 10 | empty/outside, occlusion gần-xa, occlusion một phần, polygon đặc, clip wall dài, tangent/radial/degenerate, near-wall/near-tangent, tường giao nhau, đỉnh trùng/chồng lấn, input không hợp lệ |
| `closed_sights/test_closed_sights.cpp` | 6 | wrap qua 0, thứ tự, merge projection chồng, wall bị che, rỗng/degenerate/tiny, boundary không hợp lệ |
| `open_sights/test_open_sights.cpp` | 8 | empty/full coverage, wrap đóng, wrap mở, nested/trùng/unsorted, nhiều vùng mở & touching, sector/khe ≤ epsilon, interval không hợp lệ, seam gap |
| `open_sights/test_open_points.cpp` | 4 | single sector, multiple & wrapping, full circle & determinism, danh sách rỗng & input sai |
| `pipeline/test_pipeline.cpp` | 14 | case A–H, fully enclosed, concave U, narrow obstacle, translation/rotation/scale/order, oracle 12.000 tia, invalid input & stateless repeatability |

Tổng: **7 + 10 + 6 + 8 + 4 + 14 = 49 test**.

Kết quả hiện tại (chạy `ctest` trên build hiện có):

```
100% tests passed, 0 tests failed out of 49
```

tức **49/49** test Module 1 pass.

### 11.1 Oracle độc lập độ phân giải cao

`pipeline/test_pipeline.cpp` có `oracle_hit(...)` — một oracle **hoàn toàn độc lập**: giải trực
tiếp hệ 2×2 cho tia/đoạn, **không** dùng helper production, **không** chia critical-angle,
**không** clip. Test `DeterministicRandomWallsMatchIndependentRayOracle`:

- Sinh 30 bản đồ seeded (`std::mt19937 generator(1729)`), mỗi bản đồ 8 wall ngẫu nhiên.
- Với mỗi bản đồ, quét **400 hướng** → tổng **12.000 tia**.
- So sánh `oracle_hit` trên biên đã nhận thức với `oracle_hit` trên biên gốc: kiểm tra
  "có/không bị chặn" khớp, và độ sâu tia khớp trong `1e-8`.

Mục đích: vì production dùng critical-angle chứ không quét mẫu, oracle quét mẫu (chỉ trong
verification, **không bao giờ** trong production) cung cấp một đường kiểm chứng độc lập rằng
kết quả perception trùng với "sự thật hình học" của bài toán tia.

---

## 12. Cách build và chạy Module 1 độc lập

Từ thư mục gốc repository `MARS_Modular/` (CMake ≥ 3.16, compiler C++17):

### 12.1 Build kèm test

```bash
cd ~/workspace/IVS/JetTank_runtime_bundle_win/hiwonder/MARS_Modular

cmake -S 01_perception_geometry \
      -B .build/core \
      -DBUILD_TESTING=ON

cmake --build .build/core -j2

ctest --test-dir .build/core --output-on-failure
```

Giải thích từng lệnh:

- `cmake -S 01_perception_geometry -B .build/core -DBUILD_TESTING=ON`: configure dự án con
  Module 1 (nguồn `-S`), thư mục build `-B`, bật test (`BUILD_TESTING=ON`) để tìm GTest từ host
  (`find_package(GTest CONFIG REQUIRED)`, không tải về lúc configure).
- `cmake --build .build/core -j2`: build thư viện `mars_perception_geometry` + executable
  `perception_tests` (song song 2 job).
- `ctest --test-dir .build/core --output-on-failure`: chạy 49 test qua CTest, in log khi fail.
  Test được đăng ký bằng `gtest_discover_tests`.

Lưu ý: `-DCMAKE_BUILD_TYPE=Debug` cũng được dùng trong `README.md`
(`cmake -S 01_perception_geometry -B .build/core -DCMAKE_BUILD_TYPE=Debug`).

### 12.2 Build chỉ thư viện (không test)

Được hỗ trợ: `BUILD_TESTING` mặc định của CTest là bật, tắt bằng `-DBUILD_TESTING=OFF`.

```bash
cmake -S 01_perception_geometry \
      -B .build/module1 \
      -DBUILD_TESTING=OFF

cmake --build .build/module1 -j2
```

Khi đó GTest không bị yêu cầu và `perception_tests` không được build. Output build nằm trong
`.build/` (đã Git-ignore).

### 12.3 Dùng như thư viện con

Dự án CMake khác có thể `add_subdirectory()` và link target source-tree
`mars::perception_geometry`; include path và chuẩn C++17 được propagate (PUBLIC include
`include/` và `common/cpp/include/`).

---

## 13. Cách chạy trong toàn bộ Mode 1

Bản thân Module 1 **không cần ROS**. Nó được dùng như một thư viện trong mô phỏng headless
"Mode 1" tích hợp cả 4 module (không ROS), hoặc qua adapter ROS 1/RViz trong Module 4.

### 13.1 Build và chạy mô phỏng headless

```bash
cd ~/workspace/IVS/JetTank_runtime_bundle_win/hiwonder/MARS_Modular

cmake -S . -B .build/mars -DCMAKE_BUILD_TYPE=Debug
cmake --build .build/mars --target polygon_explore_sim -j2

.build/mars/04_demo_simulation/polygon_explore_sim --map hard_alley
```

`polygon_explore_sim` (target trong `04_demo_simulation/CMakeLists.txt`) nhận `--map
hard_alley|geogebra`, `--list`, `--help`; mặc định `hard_alley`. Thư mục map được inject qua
`MARS_MODE1_MAP_DIR`.

### 13.2 Điều gì xảy ra bên trong

```
map (occupancy → LoadedMap::obstacles, Polygon2D)
        │
        ▼
perceive(pose.position, vision_radius, map.obstacles)   ← Module 1
        │  PerceptionResult
        ▼
Module 2 (ranking, VisibilityGraph, BundleSequence)
        │
        ▼
Module 3 (Algorithm 2, PlannedPath)
        │
        ▼
fake odom + path follower + trace (Module 4)
```

Điểm nối thực tế: `04_demo_simulation/launch/mode1_mission.cpp:146` gọi
`mars::perception_geometry::perceive(...)`; `update.perception` được đẩy vào Module 2
(`manager.update`); `make_frame(...)` ghi lại các layer marker mỗi lần cảm nhận.

### 13.3 Marker output của hard_alley

Log in ra các layer sau (ví dụ frame 0):

```
Mode 1 map: hard_alley
occupancy: .../04_demo_simulation/maps/hard_alley_map.yaml  obstacles=12
stop=RETURN_REACHED follower_stuck=0 entry=3.1302 return=1.9251 saw_entry=1 saw_return=1
frame 0 event=EXPLORE pose=(0.35, 0.35)
marker vision_circle center=(0.35, 0.35) r=0.85
marker closed n=1 [0.868539 +3.91516]
marker open n=1 [4.7837 +2.36803]
marker open_points n=1 (1.15805, 0.0862724)
marker graph nodes=2 edges=1
marker bundles n=1 centers= (0.35, 0.35)
marker funnel n=0
```

Phân biệt rõ vai trò từng layer:

- `vision_circle`, `closed`, `open`, `open_points` → **Module 1**.
- `graph`, `bundles`, `funnel` → **downstream** (Module 2/3). Đây **không phải** output Module 1.

Chạy xong ghi file `mode1_<map>.svg` (ví dụ `mode1_hard_alley.svg`).

---

## 14. ROS2 / RViz2 integration

- Module 1 giữ nguyên **ROS-independent**: không có ROS include, không publish topic.
- ROS chỉ là **adapter/visualization layer**, nằm ở Module 4 hoặc adapter tương lai.
- Máy hiện tại dùng **ROS2 Humble**. Module 4 cung cấp `ros2/mars_mode1_sim/` (package
  `mode1_rviz_node.cpp`, `polygon_explore_sim.launch.py`, `mode1.rviz`) bên cạnh bản ROS 1
  tương ứng trong `ros1/mars_mode1_sim/`.
- Module 4 chuyển `PerceptionResult` (và các layer downstream) thành **visualization markers**
  cho RViz. Ví dụ, layer `vision_circle`, `closed`, `open`, `open_points` xuất phát từ output
  Module 1 nhưng được vẽ bởi node ROS trong Module 4.
- Module 1 **không tự publish ROS topic nào**.

Chi tiết adapter/topic nằm ngoài phạm vi tài liệu Module 1; xem
`04_demo_simulation/visualization/README.md` và `04_demo_simulation/ros2/mars_mode1_sim/`.

---

## 15. Đối chiếu WBS

| WBS | Implementation | File/function | Test evidence | Status |
|---|---|---|---|---|
| 1.0 | Adapter hình học, hàm thuần nhận `(center, radius, obstacles)` | `pipeline/perception_pipeline.cpp` — `perceive(...)` | `test_pipeline.cpp` (case A–H, transformation, invalid, repeatability) | Hoàn thành |
| MD1.1 Neighbor Sight | Clip biên theo đĩa `r` + giải che khuất theo critical angles | `neighbor_sight/neighbor_sight.cpp` — `compute_neighbor_sight(...)` | `test_neighbor_sight.cpp` (10 test) | Hoàn thành |
| MD1.2 Closed Sights | Chiếu biên nhìn thấy → cung chặn, merge + wrap | `closed_sights/closed_sights.cpp` — `compute_closed_sights(...)`; helper `internal/intervals.hpp` | `test_closed_sights.cpp` (6 test) | Hoàn thành |
| MD1.3 Open Sights | Phần bù vòng tròn của coverage bị chặn | `open_sights/open_sights.cpp` — `compute_open_sights(...)` | `test_open_sights.cpp` (8 test) | Hoàn thành |
| MD1.3 Open Points | Midpoint góc → toạ độ 2D trên vòng `r` | `open_sights/open_points.cpp` — `compute_open_point(s)(...)` | `test_open_points.cpp` (4 test) | Hoàn thành |

Tổng test Module 1: **49/49 pass** (`ctest --test-dir .build/core`).

---

## 16. Hạn chế và phạm vi

Chỉ ghi nhận các hạn chế được nêu trong source/README của repository:

- **Hình học thuần / point observer**: kết quả tính cho một quan sát dạng điểm. Open points là
  đại diện hình học, **không** phải bảo đảm va chạm hay an toàn chuyển động cho robot có
  footprint hữu hạn. Không có inflation, uncertainty, dự đoán động, hay mô hình che phủ cảm
  biến một phần.
- **Không điều khiển robot**: Module 1 không sinh lệnh chuyển động; goal/planning/motor thuộc
  downstream.
- **Không phụ thuộc ROS**: không topic, TF, timestamp; việc quy đổi frame/độ phân giải cảm biến
  do adapter tương lai đảm nhiệm.
- **Giả định bản đồ đầy đủ cho vùng cảm nhận**: API giả định "complete obstacle geometry"; không
  có hình học nghĩa là không gian đã-biết-tự-do. Adapter cảm biến **không được** biến vùng chưa
  biết/chưa quan sát thành vùng tự do.
- **Không biểu diễn được hướng bị chặn cô lập / số đo góc bằng 0**: point obstacle, wall hướng
  tâm (radial), tangent một điểm → không sinh blocked sector.
- **Số học xấp xỉ**: có `linear_epsilon`/`angular_epsilon`; không phải số học chính xác ở mọi
  thang đo. Nên dùng toạ độ cục bộ thang mét; offset tuyệt đối lớn có thể mất chi tiết nhỏ.
- **Hiệu năng**: worst-case `O(E³)` time / `O(E²)` space; chưa tăng tốc cho map lớn (deferred).
- **Ranking/graph/navigation thuộc downstream**: Module 1 chỉ làm perception geometry; xếp hạng
  open point, dựng graph, chọn goal, bundle, funnel là của Module 2/3.
- **Footprint robot hữu hạn**: chỉ là điểm quan sát; nếu cần footprint hữu hạn thì **không** được
  cài trong module này (không có tài liệu nào nêu cơ chế footprint).

---

## 17. Tóm tắt

Module 1 (`01_perception_geometry`) là một **thư viện C++17 thuần, ROS-independent**, cung cấp
hàm thuần `perceive(center, radius, obstacles)` → `PerceptionResult`. Nó bảo đảm cho các module
downstream:

- `NeighborSight.visible_boundaries`: biên chướng ngại **nhìn thấy được** trong tầm `r`, đã giải
  che khuất (fragment, không phải toàn bộ cạnh).
- `closed_sights`: các **cung góc bị chặn** liên tục, đã merge, sắp theo start, kèm provenance
  các fragment hỗ trợ.
- `open_sights`: các **cung góc trống** = phần bù vòng tròn của coverage bị chặn, xử lý wrap và
  seam.
- `open_points`: một **điểm đại diện** trên mỗi cung trống (midpoint góc, toạ độ trên vòng `r`,
  kèm `angle` và `sight_index`).

Toàn bộ được kiểm chứng bằng **49 test GTest độc lập ROS**, bao gồm một oracle quét-tia độc lập
12.000 tia trên 30 bản đồ seeded. Module 1 chỉ làm hình học nhận thức: xếp hạng, graph, bundle,
navigation, và visualization ROS đều thuộc các module downstream.
