# 💻 Simulação do Fred 

Este repositório contém os arquivos para a simulação do Fred, o passo a passo para instalação, e a simulação propriamente dita está abaixo. 

## Instalação
### CoppeliaSim
Tutorial de instalação para o CoppeliaSim

**1- Download CoppeliaSim EDU V4.5.1 for Ubuntu 20.04**


```shell
wget -P /tmp https://coppeliarobotics.com/files/CoppeliaSim_Edu_V4_5_1_rev4_Ubuntu20_04.tar.xz
cd /tmp && tar -xvf CoppeliaSim_Edu_V4_5_1_rev4_Ubuntu20_04.tar.xz
mv CoppeliaSim_Edu_V4_5_1_Ubuntu20_04 ~/
```

**2 - Prepare ".bashrc" for CoppeliaSim**

```shell
echo 'export COPPELIASIM_ROOT_DIR="$HOME/CoppeliaSim_Edu_V4_5_1_rev4_Ubuntu20_04"' >> ~/.bashrc && source ~/.bashrc
echo 'alias coppelia="$COPPELIASIM_ROOT_DIR/coppeliaSim.sh"' >> ~/.bashrc && source ~/.bashrc
```

**3 - Se tudo deu certo, você deve conseguir abrir o Coppelia com o comando abaixo**

```shell
coppelia
```
---
### Clone repositórios
Agora precisamos clonar os repositórios para a simulação

**1 - Na pasta `src` do catkin_ws**
```shell
cd catkin_ws/src
```

**2 - fred_montagem**
```shell
git clone https://github.com/Iniciativa-Frederico/fred_montagem.git
```

**5 - de o `catkin_make`**
```shell
cd ..
catkin_make
```

**4 - fred_scripts (home folder)**
```shell
cd 
git clone https://github.com/Iniciativa-Frederico/fred_scripts.git
```



---
## Simulação
Com tudo instalado, vamos rodar a simulação. Siga todos os passos na ordem. 

**1 - Inicie o ROS**
```shell
roscore
```

**2 - Em um novo terminal, abra o Coppelia**
```shell
coppelia
```

**3 - No Coppelia, abra o arquivo de simulação**

Na aba `File` ->  `Open scence` -> selecione a simulação (arquivo fred_simulation.ttt) que está na parta `catkin_ws/src/fred_montagem` na sua home folder

**4 - Em um novo terminal, rode o launch file**
```shell
cd fred_scripts
roslaunch ROS_simulation.launch
```

**5 - Inicie a simulação no Coppelia**

Inicie a simulação no Coppelia, as vezes na primeira vez ele da uma travada legal kkkkkkkk, se isso acontecer, vá na aba `Simulation`, cliquem em `Stop simulation` e depois rode a simulação de novo. 

**6 - Bypass na segurança do Fred**

Agora precisamos dar bypass na segurança do Fred kkkkk 

Primeio para dizer que o controle está conectado
```shell
rostopic pub /joy/controler/connected std_msgs/Bool "data: true" 
```

Segundo, para tirar ele do modo de emergência
```shell
rostopic pub /joy/controler/ps4/break std_msgs/Int16 "data: 1" 
```

E por último, para colocar em modo autônomo
```shell
rostopic pub /joy/controler/ps4/triangle std_msgs/Int16 "data: 1" 
```

Esses comandos não rodam pela simulação, porque só devem ser executados uma vez. 

---

## Observações
- Toda alteração nos scripts de simulação devem ser colocada nos scripts do repositório, isso pq não é possível fazer o versionamento da simulação. 

