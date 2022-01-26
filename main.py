from envs.px4_env import SinglePx4UavEnv
import numpy as np

if __name__ == "__main__":
    env = SinglePx4UavEnv()
    #env.step(np.array([0.5, 0.5, 0.5]))