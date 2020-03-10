# [Deep-Sort](../Paper/deep-sort%20SIMPLE%20ONLINE%20AND%20REALTIME%20TRACKING%20WITH%20A%20DEEP%20ASSOCIATION%20METRIC.pdf)
## sort with deep association metric
### 状态估计
- 八维状态空间$(u,v,\gamma,h,\dot{x},\dot{y},\dot{\gamma},\dot{h})$
  - $(u,v)$:边界框的中心位置
  - $\gamma$:长宽比
  - $h$:高度
  - $(u,v,\gamma,h)$:观测值
  - $(\dot{x},\dot{y},\dot{\gamma},\dot{h})$:图像坐标中对应的速度信息
- 卡尔曼滤波
  - 常量状态转移方程
  - 线性观测方程
### 轨迹处理
- 每个轨迹$k$，有一个保存连续未匹配成功帧数的变量$a_k$，当$a_k>A_{max}$时，删去轨迹$k$。
- 当前帧目标没有可以匹配轨迹时，新建一个轨迹假设
- 对于当前帧新建的轨迹，在紧接的三帧将轨迹分类暂定
- 只有当新轨迹在紧接的三帧都有物体进行关联才被确定是新轨迹，否则删去该轨迹
### 分配
- 运动匹配度
  - 采用马氏距离刻画运动匹配度，适合在运动不确定性较低的时候
  - **第j个detection**和**第i条轨迹**之间的运动匹配度:$d^{(1)}(i, j)=\left(\boldsymbol{d}_{j}-\boldsymbol{y}_{i}\right)^{\mathrm{T}} \boldsymbol{S}_{i}^{-1}\left(\boldsymbol{d}_{j}-\boldsymbol{y}_{i}\right)$
    - $S_i$:轨迹由kalman滤波器预测得到的在当前时刻观测空间的协方差矩阵
    - $y_i$:轨迹在当前时刻的预测观测量
    - $d_j$:第j个detection的状态$(u,v,\gamma,h)$.即实际观测值
  - 通过该马氏距离(运动匹配度)对detections进行筛选，并采用卡方分布的0.95分位点作为阈值，$t^{(1)}$=9.4877。
    - $b_{i, j}^{(1)}=\mathbb{1}\left[d^{(1)}(i, j) \leq t^{(1)}\right]$
- 表观匹配度
  - 从卡尔曼滤波框架获得的预测状态是对象位置的粗略估计，采用表达匹配度作为第二个指标
  - 每个detection:$d_j$有一个与之对应的单位向量$r_j$。本文采用预训练的CNN计算该向量
  - 使用 **detection** 和 **轨迹 最新$L_k$=100之内 的detections单位向量** 之间的最小余弦距离作为detection和track之间的表观匹配程度
    - 余弦距离:$1-\frac{AB}{|A||B|}$
    - $d^{(2)}(i, j)=\min \left\{1-\boldsymbol{r}_{j}^{\mathrm{T}} \boldsymbol{r}_{k}^{(i)} | \boldsymbol{r}_{k}^{(i)} \in \mathcal{R}_{i}\right\}$
      - $r_j$:detection对应的单位向量
      - $r_k$:轨迹里detection对应的单位向量
    - $b_{i, j}^{(2)}=\mathbb{1}\left[d^{(2)}(i, j) \leq t^{(2)}\right]$
      - 阈值$t^{(2)}$由训练集得到
- 指标融合
  - 运动匹配度对短期的预测效果好，表观匹配度对长时间丢失的轨迹预测比较好
  - 当符合两个指标时，融合的指标采用加权平均的方式进行融合。
  - $$c_{i, j}=\lambda d^{(1)}(i, j)+(1-\lambda) d^{(2)}(i, j)\tag{1}$$
    - $\lambda$超参数，$\lambda$越大，表明越重视短期预测
  - $$b_{i, j}=\prod_{m=1}^{2} b_{i, j}^{(m)}\tag{2}$$
### 级联匹配
- 目的
  - 如果一条轨迹被遮挡了一段较长的时间，kalman滤波器的不断预测中就会导致概率弥散。
  - 假设现在有两条轨迹竞争同一个detection，遮挡时间长的往往得到马氏距离更小，使detection倾向于分配给丢失时间更长的轨迹。
    - 假设本来协方差矩阵是一个正态分布，连续的预测不更新导致这个正态分布的方差越来越大，离均值欧氏距离远的点可能和之前分布中离得较近的点获得同样的马氏距离值。
  - 但是直观上，该detection应该分配给时间上最近的轨迹。
- 算法描述
  - 输入
    - 已有轨迹的索引 $\mathcal{T}=\{1, \ldots, N\}$
    - 当前帧检测出来的detection的索引 $\mathcal{D}=\{1, \ldots, M\}$
    - $A_{max}$
  1. 根据(1)计算$\boldsymbol{C}=\left[c_{i, j}\right]$
  2. 根据(2)计算$\boldsymbol{B}=\left[b_{i, j}\right]$
  3. 用$\emptyset$初始化匹配集$\boldsymbol{M}$
  4. 用$\mathcal{D}$初始化匹配集$\boldsymbol{U}$
  5. for $n \in\left\{1, \ldots, A_{\max }\right\}$ do
  6. $\qquad \mathcal{T}_{n} \leftarrow\left\{i \in \mathcal{T} | a_{i}=n\right\}$ 最近n帧中未与detection相关联的track的索引
  7. $\qquad \left[x_{i, j}\right]\leftarrow$min_cost_match($\boldsymbol{C}, \mathcal{T}_{n}, \mathcal{U}$) 根据最小成本算法计算出Tn与物体检测j关联成功产生集合$\left[x_{i, j}\right]$
  8. $\qquad \mathcal{M} \leftarrow \mathcal{M} \cup\left\{(i, j) | b_{i, j} \cdot x_{i, j}>0\right\}$ 更新M为匹配成功的(物体跟踪i,物体检测j)集合
  9. $\qquad \mathcal{U} \leftarrow \mathcal{U} \backslash\left\{j | \sum_{i} b_{i, j} \cdot x_{i, j}>0\right\}$ 在集合U中删去匹配成功的集合

