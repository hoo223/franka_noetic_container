#!/usr/bin/env python3
import numpy as np

def estimate_robot_base_in_world(file_path, gap_in_m=0.025):
    # 1. 데이터 로드 (World_X, Y, Z, Robot_X, Y, Z)
    try:
        data = np.loadtxt(file_path, delimiter=',', comments='#')
    except Exception as e:
        return f"파일 읽기 오류: {e}"

    # 데이터 분리
    P_W = data[:, :3] * gap_in_m  # 목표 (Reference)
    # P_W[:, 2] = 0.01633796 # 처음엔 주석처리해서 얻어진 z값을 대입해서 사용
    P_B = data[:, 3:]  # 입력 (Source)

    # 2. SVD 기반 변환 행렬 계산 (P_W = R * P_B + t)
    centroid_W = np.mean(P_W, axis=0)
    centroid_B = np.mean(P_B, axis=0)

    W_centered = P_W - centroid_W
    B_centered = P_B - centroid_B

    # 공분산 행렬 (이전과 방향이 반대임)
    H = B_centered.T @ W_centered
    U, S, Vt = np.linalg.svd(H)
    
    R_B_W = Vt.T @ U.T

    # Reflection 처리
    if np.linalg.det(R_B_W) < 0:
        Vt[2, :] *= -1
        R_B_W = Vt.T @ U.T

    # Translation: World 원점에서 본 로봇 베이스의 위치
    t_B_W = centroid_W - R_B_W @ centroid_B

    # 3. 오차 분석
    P_W_pred = (R_B_W @ P_B.T).T + t_B_W
    rmse = np.sqrt(np.mean(np.linalg.norm(P_W_pred - P_W, axis=1)**2))

    # 4. 결과 출력
    print("=== Robot Base Pose in World Frame ===")
    print(f"Base Position (x, y, z) in World (mm): {t_B_W}")
    
    # Z축 회전(Yaw) 추출
    yaw = np.arctan2(R_B_W[1,0], R_B_W[0,0]) * 180 / np.pi
    print(f"Base Rotation (Yaw) in World: {yaw:.4f} degrees")
    print(f"Calibration RMSE: {rmse:.4f} mm")

    # 4x4 Transformation Matrix (Base to World)
    T_B_W = np.eye(4)
    T_B_W[:3, :3] = R_B_W
    T_B_W[:3, 3] = t_B_W
    
    return T_B_W

# 사용 예시
# T_B_W = estimate_robot_base_in_world('points.txt')

# 실행 (points.txt 파일이 같은 경로에 있어야 함)
matrix = estimate_robot_base_in_world('points.txt')
print("\nHomogeneous Transformation Matrix (T_W_B):")
# numpy 배열 형태로 출력
print("T = np.array([")
for row in matrix:
    print("    [" + ", ".join(f"{val:.8e}" for val in row) + "],")
print("])")