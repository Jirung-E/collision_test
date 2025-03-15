# 충돌 알고리즘의 성능 테스트
SAT방식과 GJK(+EPA)방식의 성능을 비교한다.

## 1. 환경
- CPU: 12th Gen Intel(R) Core(TM) i5-12400F
- RAM: 16GB
- OS: Windows 11

## 2. 성능 측정 방법
다음 3가지 경우에 대해 충돌 알고리즘을 적용한다.
- AABB vs AABB
- OBB vs OBB
- OBB vs Sphere

각각의 경우에 대해 100만번의 충돌체크를 수행하고, 그 시간을 측정한다.  
안정적인 결과를 얻기 위해 10번 정도 반복하여 평균값을 취한다.  

충돌여부(true/false)만 판단하는 경우와 충돌벡터와 충돌깊이까지 구하는 경우를 각각 측정한다.  

flamegraph를 이용하여 시간이 많이 소요되는 부분을 찾아내고 최적화를 시도한다.  


## 3. 결과
### 1차
충돌여부만 판단했다.  

| 충돌체크 | AABB vs AABB | OBB vs OBB | OBB vs Sphere |
|---|---|---|---|
| SAT | 3.35ms | 101.56ms | 7.35ms |
| GJK | 67.45ms | 164.51ms | 100.37ms |

왜 OBB vs OBB가 유독 느린지?
- SAT방식과 GJK방식 모두 get_vertices()를 반복적으로 호출하고있다. 그런데 get_vertices()는 매번 박스의 확장 및 회전을 계산하기 때문에 비효율적이다.
