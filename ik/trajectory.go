package ik

// FrameStatus 标记轨迹中某一帧的求解状态。
type FrameStatus int

const (
	StatusOK               FrameStatus = iota // 正常求解
	StatusNearSingular                        // 求解成功，但构型接近奇异位形
	StatusLimitUnreachable                    // 几何可达但超出关节限位
	StatusUnreachable                         // 几何不可达（工作空间外）
)

func (s FrameStatus) String() string {
	switch s {
	case StatusOK:
		return "ok"
	case StatusNearSingular:
		return "near-singular"
	case StatusLimitUnreachable:
		return "limit-unreachable"
	case StatusUnreachable:
		return "unreachable"
	default:
		return "unknown"
	}
}

// Frame 是轨迹回放的一帧。
type Frame struct {
	Target Pose        `json:"target"` // 目标位姿
	Q      []float64   `json:"q"`      // 该帧采用的关节角（失败帧保持上一帧构型）
	Joints []Point     `json:"joints"` // 各关节位置（正运动学结果，供渲染）
	Status FrameStatus `json:"status"`
	Manip  float64     `json:"manip"` // 该帧构型的可操作度
}

// SolveTrajectory 沿目标轨迹逐帧求解逆运动学。
//
// 每帧先枚举全部几何解并按限位过滤，再用 SelectClosest 相对上一帧
// 构型选解，保证连续轨迹下构型不跳变。求解失败的帧保留上一帧关节角
// （机械臂停在原地），并打上对应的状态标记；q0 为初始关节角，传 nil
// 时以零位形作为选择参考。
func (a *Arm) SolveTrajectory(targets []Pose, q0 []float64) []Frame {
	n := len(a.Lengths)
	prev := make([]float64, n)
	if q0 != nil {
		copy(prev, q0)
	}
	frames := make([]Frame, len(targets))
	for i, t := range targets {
		f := Frame{Target: t}
		sols, err := a.Solve(t)
		switch {
		case err == ErrUnreachable:
			f.Status = StatusUnreachable
		case err == ErrLimitUnreachable:
			f.Status = StatusLimitUnreachable
		case err != nil:
			f.Status = StatusUnreachable
		default:
			q := SelectClosest(sols, prev)
			copy(prev, q)
			if a.IsSingular(q) {
				f.Status = StatusNearSingular
			} else {
				f.Status = StatusOK
			}
		}
		f.Q = append([]float64(nil), prev...)
		f.Joints, _ = a.ForwardKinematics(prev)
		f.Manip = a.Manipulability(prev)
		frames[i] = f
	}
	return frames
}
