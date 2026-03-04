# Insertion Mode Integration TODO

## 1) Controller Switcher 확장 (`scripts/controller_switcher.py`)

- [ ] `INSERTION` 모드 키 상수 추가 (예: `j`)
- [ ] `MODE_DICT`에 `INSERTION` 추가
- [ ] `print_guide()`에 insertion 항목 추가
- [ ] 퍼블리셔 추가
  - [ ] `/policy/enable` (`std_msgs/Bool`)
  - [ ] `/policy/reset` (`std_msgs/Bool`)
  - [ ] (선택) `/policy/status` 구독
- [ ] insertion 진입 로직 구현
  - [ ] `cartesian_impedance_example_controller`로 스위칭
  - [ ] `/policy/reset=True` 1회 발행
  - [ ] `/policy/enable=True` 발행
- [ ] insertion 종료 로직 구현
  - [ ] 모드 전환/종료 시 `/policy/enable=False` 발행
- [ ] `change_mode()`의 `self.mode` 업데이트 로직 정리
  - [ ] controller 모드와 action성 명령(요청/줌/고정) 구분으로 표시 일관성 확보

## 2) Policy Node 게이팅 (`peg_insert_policy/scripts/policy_node.py`)

- [ ] `/policy/enable` 구독 추가 (`std_msgs/Bool`)
- [ ] 내부 상태 변수 추가 (`policy_enabled`)
- [ ] `enable=False` 시 추론/명령 발행 중지
- [ ] `enable=True` 전환 시 초기화 정책 정의
  - [ ] reset 강제 여부
  - [ ] RNN state 유지/초기화
- [ ] `/policy/status` 발행 추가 (권장)
  - [ ] 상태 예: `IDLE`, `RUNNING`, `TF_MISSING`, `ERROR`
- [ ] TF 미수신/예외 시 안전 동작 명시
  - [ ] 명령 발행 중지 + 상태 전이

## 3) 관측/행동 정합 (학습 환경 <-> 실기)

- [ ] 프레임 기본값 정합
  - [ ] `ee_frame` 기본값을 `panda_hand_tcp`로 검토
  - [ ] `object_frame`을 `insert_center_part11` 또는 `/target_pose_frame` 연동으로 정리
- [ ] `action_target_frame` 파라미터를 실제 코드에 반영 (`tcp`/`plug`)
- [ ] 파라미터 실측 캘리브레이션
  - [ ] `tcp_to_plug_pos`
  - [ ] `tcp_to_plug_quat_xyzw`
  - [ ] `object_to_goal_plug_pos`
  - [ ] `object_to_goal_plug_quat_xyzw`
- [ ] 안전 제한값 점검
  - [ ] `max_translation_step`
  - [ ] `max_rotation_step_rad`
  - [ ] `workspace_min_xyz`, `workspace_max_xyz`

## 4) Launch / Runtime 결선

- [ ] `policy.launch`를 시스템 bringup에 통합할지 결정
  - [ ] 옵션 A: `franka_main.launch`에서 include
  - [ ] 옵션 B: 별도 launch 병행
- [ ] launch arg 외부화
  - [ ] `checkpoint_path`
  - [ ] `train_config_path`
  - [ ] 프레임/토픽 파라미터
- [ ] 다중 워크스페이스 환경 변수/overlay 확인
  - [ ] `franka_interface`와 `peg_insert_policy` 동시 인식 확인

## 5) 안정성/운영성

- [ ] insertion 최대 실행시간 타임아웃 추가 (예: 10~20초)
- [ ] 정책 노드 heartbeat/log 주기 추가 (선택)
- [ ] clamp 발생/TF 실패 카운트 metric 추가
- [ ] 비정상 상태에서 자동 disable 전략 확정

## 6) 검증 계획

### A. 단독 테스트 (`peg_insert_policy`)

- [ ] `policy_test.launch`로 enable on/off 반응 확인
- [ ] `/cartesian_impedance_example_controller/equilibrium_pose` 발행 주기 확인
- [ ] reset 이후 내부 상태 초기화 확인

### B. 통합 테스트 (실기 스택)

- [ ] insertion 모드 진입 시 컨트롤러가 impedance인지 확인
- [ ] insertion 모드 진입 시 `/policy/reset=True` 1회 확인
- [ ] insertion 모드 유지 중 `/policy/enable=True` 유지 확인
- [ ] 모드 이탈 시 `/policy/enable=False` 확인

### C. 실패 시나리오

- [ ] TF 누락 시 안전 정지
- [ ] object frame 누락 시 명령 미발행
- [ ] checkpoint 로드 실패 시 노드 상태/에러 로그 확인

## 7) 완료 기준 (Definition of Done)

- [ ] insertion 모드 키 입력으로 정책 기반 삽입 제어가 시작/중지된다
- [ ] 모드 전환 시 정책 enable/disable이 확실히 동작한다
- [ ] TF 누락/오류 상황에서 로봇이 안전하게 정지한다
- [ ] 단독 테스트 + 통합 테스트 시나리오를 모두 통과한다
