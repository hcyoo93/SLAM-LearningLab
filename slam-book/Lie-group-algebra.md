# Lie Group and Lie Algebra

#### 왜 배우는가?

#### 목적: 최적화 문제의 단순화

- 3D SLAM 문제는 비선형최적화를 통해 optimal camera pose **T**=[**R**|**t**]를 찾는 과정이 필요.
- Rotation matrix **R**의 특징(orthogonal, determinant=1) 때문에 제약조건이 있는 최적화 문제로 구성이 되어 풀기가 어려움.
- Lie Group과 Lie Algebra 간의 변환 관계를 이용하면, 제약조건이 없는 최적화 문제로 보다 쉽게 풀이가 가능.



#### Table of contents

1. [Basics of Lie Group and Lie Algebra](#Basics of Lie Group and Lie Algebra)
2. Exponential and Logarithmic Mapping
3. Lie Algebra Derivation and Perturbation Model
4. Similar Transform Group and Its Lie Algebra



## Basics of Lie Group and Lie Algebra

#### 무엇인가?

#### 1. Group의 정의

**집합과 연산으로 구성된 대수적 구조**로 아래의 그룹 공리(group axiom)를 만족해야한다.
$$
\forall a_{1}, a_{2} \in A, a_{1} \cdot a_{2} \in A \\
\forall a_{1}, a_{2}, a_{3} \in A,\left(a_{1} \cdot a_{2}\right) \cdot a_{3}=a_{1} \cdot\left(a_{2} \cdot a_{3}\right) \\
\exists a_{0} \in A, s.t. \forall a \in A, a_{0} \cdot a=a \cdot a_{0}=a \\
\forall a \in A, \exists a^{-1} \in A, st\; a \cdot a^{-1}=a_{0}
$$

각각은 닫힘성, 결합법칙, 항등원, 역원을 의미한다. 

#### 2. Lie Group

생각보다 정리에 시간이 걸려서... 발표는 ppt로 엮어서 진행하고 gitbook은 추후 업데이트하겠습니다

쥬륵...

유희철 올림.
