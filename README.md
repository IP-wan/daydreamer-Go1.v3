# daydreamer-Go1 with free-dog-sdk

### 1. Устанавливаем систему управления пакетами conda
### 2. Клонируем репозиторий с github
```
https://github.com/IP-wan/daydreamer-Go1.v3.git
```
### 3. Переходим в директорию
```
cd daydreamer-Go1.v3
```
### 4. Создаем виртуальное окружение
```
conda create -n <name env>
```
### 5. Активируем виртуальное окружение
```
conda activate <name env>
```
### 6. Устанавливаем версию python
```
conda install python==3.10
```
### 7. Устанавливаем зависимости из файла requirements
```
pip install -r requirements.txt 
```
### 8. В терминаторе запускаем 2 процесса 
```
CUDA_VISIBLE_DEVICES=0 python embodied/agents/dreamerv2plus/train.py --configs go1 --task go1_sim --run learning --tf.platform gpu --logdir ~/logdir/run1

CUDA_VISIBLE_DEVICES=0 python embodied/agents/dreamerv2plus/train.py --configs go1 --task go1_real --run acting --tf.platform gpu --env.kbreset True --imag_horizon 1 --replay_chunk 8 --replay_fixed.minlen 32 --imag_horizon 1 --logdir ~/logdir/run1
```