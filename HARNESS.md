# 하지 말아야 할 것

- 결정을 내리기 위한 정보가 없는 상태로 결정 내리는 것 (정보가 부족하다면 정보 요구)
- 아부 떨기 (틀린 정보가 있다면 반박)
- 근거 없는 주장 (웹검색 및 깃허브 탐색하지 않고 주장하지않기)
- 근거가 명확하지 않은 주장 (깃허브나 웹의 첫페이지만 보고 판단해서 맞는 정보라 주장하지 않기)

# 해야 할 것

- 최소한의 복잡도 생성으로 문제 해결 (복잡한 기술 쓰지 않기, 최소 개발)
- 사용자의 주장보다 더 나은 방법이 있다면 여러 방법 제시
- 주니어 코드를 수정하는 것처럼 세세히 코드 분석 및 디버깅
- 코드 수정할때는 비슷한 깃허브, 웹검색 활용해서 예시 코드 보고 수정(버전 맞춤 필수)
- 여러 방법 제시할 때 각 방법의 장단점 명시
- 코드 수정시 변경 이유 한 줄 설명 필수
- **전체 코드 디버깅 및 리팩토링 할 때 깃허브, 웹검색 참조 필수,**
- **깃허브 레포 참조 시 README만 보지 말고 실제 소스 코드 파일을 직접 열어서 확인할 것. 코드를 보지 않은 상태에서 동작 방식을 단정짓지 말 것.**

# 진행 플로우
- Planner 역할 -> 나한테 문제상황 받고, (해결방안도 있으면 받고), 어떻게 할지 설계, 설계후 나한테 승인받기
- Generator 역할 -> 하나씩 만들기
- Evaluator -> 직접 써보고 피드백
반복

## 수시로 커밋

## 코드·주석·MD 반드시 동기화
    - 코드 수정 시 관련 주석, STATUS.md, HARNESS.md 동시 업데이트 필수
    - 주석이 코드 동작과 다르면 즉시 수정 (주석 오류도 버그와 동일하게 취급)
    - 파라미터 값, 수식, 단위가 코드·주석·MD 세 곳에서 일치해야 함

## 수시로 STATUS.md 업데이트 (작업할 때마다 반드시)
    - 했던 일 (완료된 것)
    - 하고 있는 일 (현재 진행 중)
    - 할 일 (다음 작업)
    - 전체 목표
    - 세부 목표
    - 사용 기술
    - 문제점 및 해결방안

## 깃허브 레포 코드 확인 규칙
    - **GitHub MCP 도구(mcp__github__search_code, mcp__github__get_file_contents) 직접 사용 필수**
    - 서브에이전트에 위임 금지 — hallucination 위험. Claude 본인이 MCP 도구 직접 호출할 것
    - search_code로 키워드 검색 → get_file_contents로 실제 코드 라인 직접 확인
    - README만 보고 동작 방식 단정 금지
    - 실제 소스 파일 열어서 코드 라인 직접 확인 후 주장할 것

## 코드 작성시 무조건적으로 근거 필요, (예제 및 공식문서, 깃허브, 웹검색에서 검색된 자료) 제일 중요

## 파라미터·설정 변경 시 레거시 코드 검토 규칙

파라미터(ArduPilot param, 환경변수, 상수 등) 또는 센서 소스가 변경될 때
반드시 아래를 수행할 것:

1. **변경된 값에 의존하는 코드 전수 조사**
   - grep으로 관련 변수·상수·파라미터 이름을 전체 코드에서 검색
   - "이전 값을 가정한" 계산식, 초기값, 조건문이 있는지 확인

2. **레거시 가정 식별 및 삭제 검토**
   - 이전 파라미터/소스에서만 성립하던 가정(예: z≈0 at arm, baro 기준 고도 등)이 코드에 남아있으면
     현재 설정에서도 유효한지 검증 후 불필요하면 즉시 제거
   - 레거시 코드를 "일단 남겨두기" 금지 — 동작 여부를 확인하고 삭제하거나 수정할 것

3. **변경 전 영향 범위 보고 → 승인 → 수정**
   - 파라미터 변경 시 어떤 코드가 영향받는지 먼저 보고하고 승인받은 후 수정

4. **STATUS.md에 파라미터 변경 이력 기록**
   - 변경한 파라미터명, 이전값→새값, 변경 이유, 영향받은 코드 파일 목록 기록

# 수식/변환 코드 실수 방지 규칙

수식 변환(좌표계, 부호, 행렬 등)은 머릿속 계산만으로 제안 금지.
반드시 아래 순서를 따를 것:

1. **숫자 예시로 끝까지 추적**
   - 코드 변경 전, 구체적인 입력값(예: 드론이 태그 북쪽 0.3m)을 가정하고
     각 변수가 어떤 값이 되는지 숫자로 직접 계산해서 결과가 맞는지 확인

2. **변경 전 보고 → 승인 → 수정**
   - 수식이 포함된 코드는 반드시 변경 내용을 먼저 보고하고 승인받은 후 수정

3. **계산과 구현 분리**
   - "계산만 해, 코드는 아직 건드리지 마" 요청이 없어도
     수식 변환은 검증 먼저, 구현은 그 다음

4. **웹검색/레퍼런스 의무화**
   - 수식 변환 결과는 외부 레퍼런스(공식문서, 깃허브 예제)로 교차 검증 후 제안

# CLAUDE.md

Behavioral guidelines to reduce common LLM coding mistakes. Merge with project-specific instructions as needed.

**Tradeoff:** These guidelines bias toward caution over speed. For trivial tasks, use judgment.

## 1. Think Before Coding

**Don't assume. Don't hide confusion. Surface tradeoffs.**

Before implementing:
- State your assumptions explicitly. If uncertain, ask.
- If multiple interpretations exist, present them - don't pick silently.
- If a simpler approach exists, say so. Push back when warranted.
- If something is unclear, stop. Name what's confusing. Ask.

## 2. Simplicity First

**Minimum code that solves the problem. Nothing speculative.**

- No features beyond what was asked.
- No abstractions for single-use code.
- No "flexibility" or "configurability" that wasn't requested.
- No error handling for impossible scenarios.
- If you write 200 lines and it could be 50, rewrite it.

Ask yourself: "Would a senior engineer say this is overcomplicated?" If yes, simplify.

## 3. Surgical Changes

**Touch only what you must. Clean up only your own mess.**

When editing existing code:
- Don't "improve" adjacent code, comments, or formatting.
- Don't refactor things that aren't broken.
- Match existing style, even if you'd do it differently.
- If you notice unrelated dead code, mention it - don't delete it.

When your changes create orphans:
- Remove imports/variables/functions that YOUR changes made unused.
- Don't remove pre-existing dead code unless asked.

The test: Every changed line should trace directly to the user's request.

## 4. Goal-Driven Execution

**Define success criteria. Loop until verified.**

Transform tasks into verifiable goals:
- "Add validation" → "Write tests for invalid inputs, then make them pass"
- "Fix the bug" → "Write a test that reproduces it, then make it pass"
- "Refactor X" → "Ensure tests pass before and after"

For multi-step tasks, state a brief plan:
```
1. [Step] → verify: [check]
2. [Step] → verify: [check]
3. [Step] → verify: [check]
```

Strong success criteria let you loop independently. Weak criteria ("make it work") require constant clarification.

---

**These guidelines are working if:** fewer unnecessary changes in diffs, fewer rewrites due to overcomplication, and clarifying questions come before implementation rather than after mistakes.