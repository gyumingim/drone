"""tag_test.py — AprilTag 감지 테스트 (비행 없음)

실행: python3 tag_test.py
  - 카메라 화면 + 감지된 tag pose 출력
  - q 키 또는 Ctrl+C 종료
"""
import math
import time
import cv2
from tag_reader import TagReader, TAG_SIZE_M

print(f'[TEST] tag_size={TAG_SIZE_M}m  |  q 키로 종료')
reader = TagReader()
reader.start()

print('[TEST] 카메라 초기화 대기...')
time.sleep(2)

try:
    while True:
        frame = reader.get_frame()
        pose  = reader.get_pose()

        if frame is not None:
            cv2.imshow('AprilTag Test', frame)

        if pose:
            n, e, d, yaw = pose
            print(f'[TAG] N={n:+.3f}m  E={e:+.3f}m  D={d:.3f}m  '
                  f'yaw={math.degrees(yaw):+.1f}deg  '
                  f'dist2d={math.hypot(n, e):.3f}m')
        else:
            print('[TAG] 미감지')

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        time.sleep(0.1)

except KeyboardInterrupt:
    pass
finally:
    cv2.destroyAllWindows()
    print('[TEST] 종료')
