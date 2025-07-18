# ConeSTELLATION 개발 계획

## 개요
ConeSTELLATION은 GLIM의 모듈식 아키텍처를 참고하여 개발하는 콘 기반 Graph SLAM 시스템입니다. LiDAR 포인트 클라우드 대신 콘 클러스터링 인스턴스를 입력으로 받아, YOLO로 라바콘 색상을 추가한 후 SLAM을 수행합니다.

## 핵심 아키텍처 (GLIM 참고)

### 1. 모듈 구조
```
cone_stellation/
├── include/cone_stellation/
│   ├── common/              # 공통 데이터 구조 및 유틸리티
│   ├── preprocessing/       # 콘 데이터 전처리
│   ├── odometry/           # 콘 기반 포즈 추정 (내부적으로 데이터 연관 포함)
│   ├── mapping/            # 로컬 및 글로벌 맵 구축 (루프 클로저 포함)
│   ├── util/               # 설정, 로깅, 직렬화
│   └── viewer/             # 시각화
├── src/
│   └── cone_stellation/     # 구현 파일
├── config/                 # JSON 설정 파일
├── modules/                # 동적 로딩 가능한 모듈
└── scripts/                # 시뮬레이션 및 테스트 스크립트 (cc_slam_sym에서 가져옴)
```

### 2. 핵심 베이스 클래스 (GLIM 패턴 적용)

#### 2.1 ConePreprocessorBase
```cpp
// GLIM의 PreprocessingModule 패턴 적용
class ConePreprocessorBase {
public:
  virtual ~ConePreprocessorBase() = default;
  virtual ConeCloud::Ptr process(const ConeDetections& raw_cones) = 0;
  virtual void set_config(const Config& config) = 0;
};
```

#### 2.2 ConeOdometryBase
```cpp
// GLIM의 OdometryEstimationBase 패턴 적용
class ConeOdometryBase {
public:
  virtual ~ConeOdometryBase() = default;
  virtual ConeFrame::Ptr estimate(const ConeCloud::Ptr& cones) = 0;
  virtual bool requires_odom() const = 0;
  virtual void insert_odom(const nav_msgs::msg::Odometry& odom) = 0;
  
  // 내부적으로 콘 데이터 연관 수행
  // GLIM처럼 factor 내부에서 correspondence 찾기
};
```

#### 2.3 LocalMapBuilderBase
```cpp
// GLIM의 SubMappingBase 패턴 적용
class LocalMapBuilderBase {
public:
  virtual ~LocalMapBuilderBase() = default;
  virtual void insert_frame(const ConeFrame::Ptr& frame) = 0;
  virtual std::vector<ConeMap::Ptr> get_maps() = 0;
};
```

#### 2.4 GlobalMapperBase
```cpp
// GLIM의 GlobalMappingBase 패턴 적용
class GlobalMapperBase {
public:
  virtual ~GlobalMapperBase() = default;
  virtual void insert_map(const ConeMap::Ptr& map) = 0;
  virtual void optimize() = 0;
  virtual void save(const std::string& path) = 0;
};
```

### 3. 핵심 데이터 구조

#### 3.1 Cone
```cpp
struct Cone {
  Eigen::Vector3d position;      // 3D 위치
  ConeColor color;               // YOLO로 검출된 색상
  double confidence;             // 검출 신뢰도
  int id;                       // 고유 ID
  double timestamp;             // 타임스탬프
};
```

#### 3.2 ConeFrame
```cpp
// GLIM의 EstimationFrame 패턴 적용
struct ConeFrame {
  using Ptr = std::shared_ptr<ConeFrame>;
  
  std::vector<Cone> cones;                    // 검출된 콘들
  Eigen::Isometry3d T_world_robot;           // 로봇 포즈
  double timestamp;                          // 타임스탬프
  std::unordered_map<int, int> associations; // 콘 ID -> 랜드마크 ID
  
  // GLIM처럼 custom data 지원
  std::unordered_map<std::string, std::shared_ptr<void>> custom_data;
};
```

#### 3.3 ConeMap
```cpp
// GLIM의 SubMap 패턴 적용
struct ConeMap {
  using Ptr = std::shared_ptr<ConeMap>;
  
  std::unordered_map<int, Cone> landmarks;   // 랜드마크 콘들
  std::vector<ConeFrame::Ptr> frames;        // 구성 프레임들
  Eigen::Isometry3d origin;                  // 맵 원점
};
```

### 4. 구현 전략

#### Phase 1: 기본 인프라 구축 (1-2주)
1. **디렉토리 구조 생성**
2. **기본 데이터 구조 구현** (Cone, ConeFrame, ConeMap)
3. **베이스 클래스 인터페이스 정의**
4. **CMakeLists.txt 구성** (GLIM 참고)
5. **JSON 설정 시스템 구현** (GLIM의 Config 클래스 차용)

#### Phase 1.5: 시뮬레이션 통합 (3일)
1. **cc_slam_sym 시뮬레이션 재사용**
   - dummy_publisher_node.py 복사 및 수정
   - sensor_simulator.py, motion_controller.py 재사용
   - cone_definitions.py 그대로 사용
   - 필요한 custom_interface 메시지 확인

#### Phase 2: 핵심 모듈 구현 (2-3주)
1. **ConePreprocessor 구현**
   - 노이즈 필터링
   - 중복 제거
   - 색상 검증
   
2. **ConeOdometry 구현**
   - 콘 기반 포즈 추정
   - 내부적으로 콘 매칭/연관 수행
   - Mahalanobis 거리 기반
   
3. **LocalMapBuilder 구현**
   - 키프레임 관리
   - 로컬 맵 생성
   - 콘 랜드마크 관리

#### Phase 3: GTSAM 통합 (2주)
1. **Factor 구현**
   - OdometryFactor (GLIM 참고)
   - ConeObservationFactor
   - PriorFactor
   
2. **GlobalMapper 구현**
   - GTSAM 그래프 구축
   - 최적화 실행
   - 결과 추출

#### Phase 4: 비동기 처리 및 최적화 (1주)
1. **AsyncConeAssociation** (GLIM의 AsyncOdometryEstimation 참고)
2. **AsyncLocalMapBuilder** (GLIM의 AsyncSubMapping 참고)
3. **AsyncGlobalMapper** (GLIM의 AsyncGlobalMapping 참고)
4. **스레드 안전성 보장**

#### Phase 5: ROS2 통합 (1주)
1. **ROS2 노드 구현**
2. **토픽 인터페이스 정의**
3. **서비스 및 파라미터 구현**
4. **Launch 파일 작성**

#### Phase 6: 시각화 및 디버깅 (1주)
1. **ConeViewer 구현** (GLIM의 viewer 모듈 참고)
2. **RViz 플러그인 개발**
3. **디버깅 도구 구현**

### 5. GLIM에서 차용할 핵심 요소

1. **모듈 동적 로딩 시스템**
   ```cpp
   // GLIM의 load_module 함수 패턴
   template<typename T>
   std::shared_ptr<T> load_module(const std::string& module_name);
   ```

2. **콜백 시스템**
   ```cpp
   // GLIM의 CallbackSlot 패턴
   template<typename Func>
   class CallbackSlot;
   ```

3. **설정 시스템**
   ```cpp
   // GLIM의 Config 클래스 활용
   class Config {
     json data;
     template<typename T>
     T param(const std::string& key, const T& default_value);
   };
   ```

4. **비동기 래퍼 패턴**
   ```cpp
   // GLIM의 Async 래퍼 패턴
   template<typename T>
   class AsyncWrapper {
     std::thread thread;
     ConcurrentQueue<typename T::InputType> input_queue;
   };
   ```

### 6. 주요 차이점 (GLIM vs ConeSTELLATION)

| 항목 | GLIM | ConeSTELLATION |
|------|------|----------------|
| 입력 데이터 | 포인트 클라우드 | 콘 검출 결과 (TrackedConeArray) |
| 맵 표현 | Voxel Map | 콘 랜드마크 맵 |
| 등록 방법 | ICP/GICP (내부 대응점) | 콘 매칭 (Factor 내부) |
| 센서 | LiDAR + IMU | LiDAR + Camera + Odom |
| 데이터 연관 | Factor 내부에서 암묵적 | ConeObservationFactor 내부 |

### 7. 예상 도전 과제 및 해결 방안

1. **데이터 연관 문제**
   - 문제: 콘 오검출 및 미검출
   - 해결: Robust한 연관 알고리즘, 색상 정보 활용

2. **스케일 문제**
   - 문제: 적은 수의 랜드마크
   - 해결: 효율적인 그래프 구조, 슬라이딩 윈도우

3. **실시간 처리**
   - 문제: 연산 복잡도
   - 해결: 비동기 처리, GPU 활용 고려

### 8. 테스트 전략

1. **단위 테스트**: 각 모듈별 독립적 테스트
2. **통합 테스트**: 전체 파이프라인 테스트
3. **시뮬레이션 테스트**: 가상 콘 데이터로 테스트
4. **실제 데이터 테스트**: rosbag 활용

## 결론

ConeSTELLATION은 GLIM의 검증된 아키텍처를 콘 기반 SLAM에 맞게 적용하여, 모듈성, 확장성, 성능을 모두 갖춘 시스템으로 개발할 예정입니다. GLIM의 핵심 설계 원칙을 따르면서도 콘 기반 SLAM의 특수성을 고려한 최적화를 진행할 것입니다.