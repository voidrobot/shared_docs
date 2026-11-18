# 실행 기록 CSV

`./run_demo.sh --record`로 실행하면, 프로그램이 끝날 때 `logs/<시각>_` 아래에 CSV 세 개가 생깁니다. Ctrl-C로 멈춰도
저장됩니다. `--record`가 없어도 `_bridge.npz`와 `_overlay.png`는 항상 생깁니다.

| 파일 | 주기 | 한 줄이 뜻하는 것 | 기록 구간 |
|---|---|---|---|
| `_policy.csv` | 30 Hz | 정책 한 스텝 | 큐브를 돌리는 동안(warmup, drive)만 |
| `_state.csv` | 100 Hz | 드라이버가 보낸 `/joint_states` 메시지 하나 | 시작부터 끝까지 |
| `_cmd.csv` | 100 Hz | 손에 보낸 `/joint_cmd` 하나 | 시작부터 끝까지 |

## 공통 규칙

- **시계가 하나입니다.** 세 파일의 `t`는 모두 이 PC의 `time.monotonic()`에서 같은 `t0`를 뺀 초입니다. `t0`는 손이
  휴식 자세로 옮겨 가기 시작한 순간이고, 그 전에 받은 상태는 음수로 기록됩니다. 그래서 세 파일을 따로 맞추지 않고
  바로 겹쳐 그릴 수 있습니다.
- **관절 순서는 `JOINT_NAMES`입니다.** `joint00, joint01, joint02, joint03, joint10, ..., joint43` 순서로, 손가락
  번호(0 엄지, 1 검지, 2 중지, 3 약지, 4 새끼) 다음에 마디 번호(0–3)가 옵니다. 정책 내부의 breadth-first 순서가 아닙니다.
- 단위는 각도 rad, 각속도 rad/s, 토크 Nm, 시간 s입니다.

## `_policy.csv` (30 Hz, 46열)

| 열 | 뜻 |
|---|---|
| `t` | 이 스텝이 돈 100 Hz tick이 시작한 시각입니다. `target`은 이 시각부터 `Command`에 적용됩니다 |
| `t_act` | action 계산이 끝나고 `Command`에 넣은 시각입니다 |
| `t_state` | 정책이 읽은 `/joint_states` 메시지를 받은 시각입니다 |
| `t_frame` | 정책이 쓴 카메라 프레임이 스트림에서 도착한 시각입니다(태그 검출 전) |
| `t_exposure` | 그 프레임을 찍은 시각입니다. 카메라 자체 stamp를 벽시계(`time.time()`)를 거쳐 monotonic 시계로 옮긴 값입니다 |
| `stage` | `1`은 면을 맞추는 정책, `2`는 자리를 잡고 멈추는 정책입니다 |
| `target_<joint>` × 20 | 정책이 낸 목표 관절 위치(desired position)입니다. EMA, 속도 제한, clamp를 거치기 **전** 값입니다 |
| `action_<joint>` × 20 | 정책 네트워크의 원래 출력입니다. 크기 제한이 없습니다 |

`target`이 만들어지는 방식은 단계에 따라 다릅니다.

- stage 1: `target = clip(REST + 0.5 × clip(action, -1, 1), soft limit)`. 다만 release 뒤 다시 움직이기 시작할 때는
  `--start_s` 동안 action을 grip 쪽으로 섞은 값(`grip`)으로 계산합니다.
- stage 2: 직전 target에 `0.02 × action`을 더하고, 측정된 관절 ±0.145 rad 안으로 묶은 값입니다.

### 한 스텝의 시각 순서

```
t_exposure ──▶ t_frame ──┐
                          ├──▶ t ──(추론)──▶ t_act ──▶ /joint_cmd 전송 (같은 tick)
              t_state ───┘
```

언제나 `t_exposure ≤ t_frame ≤ t`, `t_state ≤ t ≤ t_act`가 성립합니다. 이 열들로 다음 값을 계산합니다.

| 계산 | 뜻 |
|---|---|
| `t - t_exposure` | 카메라 지연: 찍은 순간부터 정책이 쓰기까지 |
| `t - t_frame` | 프레임이 도착한 뒤 태그 검출과 다음 tick을 기다린 시간 |
| `t - t_state` | 정책이 읽은 관절 상태의 나이 |
| `t_act - t` | 추론(관측 계산 + 네트워크) 시간 |

## `_state.csv` (100 Hz, 62열)

| 열 | 뜻 |
|---|---|
| `t` | 메시지를 **받은** 시각(이 PC의 monotonic 시계)입니다 |
| `stamp` | 드라이버가 메시지 헤더에 찍은 시각(ROS 시계, 벽시계 기준 초)입니다. `t`와 기준점이 다르므로 빼서 비교하지 않고, 간격만 봅니다 |
| `pos_<joint>` × 20 | 관절 위치 |
| `vel_<joint>` × 20 | 드라이버가 보낸 관절 속도 |
| `eff_<joint>` × 20 | 드라이버가 보낸 토크 |

형식은 개발 저장소의 `scripts/hand_test/real_play.py`와 같아서 `sysid.py`가 그대로 읽습니다.

## `_cmd.csv` (100 Hz, 21열)

| 열 | 뜻 |
|---|---|
| `t` | 명령을 보낸 시각입니다 |
| `<joint>` × 20 | 실제로 보낸 관절 명령입니다 |

30 Hz `target`이 100 Hz 명령이 되는 과정은 `command.py`의 `Command`가 맡습니다.

1. **EMA**: 시뮬레이터의 physics 주기(120 Hz)에 맞춰 `ema = 0.5·target + 0.5·ema`를 계산합니다. 시간 상수는 약 12 ms입니다.
2. **속도 제한**: tick마다 `vmax / 100`(기본 1.2 rad/s → 0.012 rad)까지만 움직입니다. 큰 점프를 경사로 바꾸는 것이 이 단계입니다.
3. **clamp**: 측정된 관절 위치 ±0.145 rad 안으로 자릅니다.
4. 마지막으로 hard limit으로 한 번 더 자른 뒤 보냅니다.

## 정상인지 확인하는 방법

```python
import numpy as np

name = "logs/2026-11-18_13-33-46"
pol = np.genfromtxt(f"{name}_policy.csv", delimiter=",", names=True)
state = np.genfromtxt(f"{name}_state.csv", delimiter=",", names=True)
cmd = np.genfromtxt(f"{name}_cmd.csv", delimiter=",", names=True)

ms = lambda x: np.percentile(1e3 * x, [1, 50, 99]).round(1)
print("policy dt", ms(np.diff(pol["t"])), "state dt", ms(np.diff(state["t"])), "cmd dt", ms(np.diff(cmd["t"])))
print("camera latency", ms(pol["t"] - pol["t_exposure"]), "inference", ms(pol["t_act"] - pol["t"]))
```

2026-11-18 13:33 실행(정책 1812스텝, 60초)의 값이 기준입니다.

| 항목 | p1 / p50 / p99 |
|---|---|
| 정책 간격 | 29.8 / 30.0 / 40.2 ms |
| `/joint_states` 간격 | 8.9 / 10.0 / 11.1 ms |
| `/joint_cmd` 간격 | 4.4 / 10.0 / 15.6 ms |
| 카메라 지연 `t - t_exposure` | 25.6 / 31.6 / 37.3 ms |
| 관절 상태의 나이 `t - t_state` | 1.5 / 2.7 / 3.1 ms |
| 추론 `t_act - t` | 3.7 / 5.3 / 5.8 ms |

**`/joint_cmd` 간격이 고르지 않은 것은 정상입니다.** 정책이 도는 tick에서는 추론을 마친 뒤 명령을 보내므로 약 5 ms
늦게 나가고, 다음 tick은 원래 일정대로 나가서 간격이 약 15 ms와 5 ms로 나뉩니다. 평균 주기는 100 Hz로 유지됩니다.
