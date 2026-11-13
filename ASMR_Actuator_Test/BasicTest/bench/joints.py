"""관절별 CAN node ID와 모터 종류.

수업 문서에는 번호 규칙이 없어서 원본 Berkeley Humanoid Lite 저수준 코드
(berkeley-humanoid-lite-lowlevel, robot/humanoid.py)의 다리 관절 번호를 그대로 쓴다.
왼쪽 다리는 홀수, 오른쪽 다리는 짝수. 9·10은 원본에서 팔 관절 번호라 비워 둔다.
0은 쓰지 않으므로 설정 안 된 새 보드(기본 node 0)와 겹치지 않는다.
관절 이름은 수업 Robot Model (v1) 이름, 모터 배치는 같은 문서의 표.
"""

JOINTS = [
    # (관절 이름, node ID, 모터)
    ("leg_left_hip_roll_joint", 1, "6512"),
    ("leg_left_hip_yaw_joint", 3, "6512"),
    ("leg_left_hip_pitch_joint", 5, "6512"),
    ("leg_left_knee_pitch_joint", 7, "6512"),
    ("leg_left_ankle_pitch_joint", 11, "5010"),
    ("leg_left_ankle_roll_joint", 13, "5010"),
    ("leg_right_hip_roll_joint", 2, "6512"),
    ("leg_right_hip_yaw_joint", 4, "6512"),
    ("leg_right_hip_pitch_joint", 6, "6512"),
    ("leg_right_knee_pitch_joint", 8, "6512"),
    ("leg_right_ankle_pitch_joint", 12, "5010"),
    ("leg_right_ankle_roll_joint", 14, "5010"),
]

AXIS1_NODE_ID = 63  # 안 쓰는 axis1 (xdrive_config.txt)


def find(query):
    """'left_hip_roll' 또는 'leg_left_hip_roll_joint' -> (이름, node, 모터)."""
    for j in JOINTS:
        if j[0] in (query, f"leg_{query}_joint"):
            return j
    names = ", ".join(j[0][4:-6] for j in JOINTS)
    raise SystemExit(f"관절 '{query}'를 모릅니다. 가능한 값: {names}")


def by_node(node):
    return next((j for j in JOINTS if j[1] == node), None)
