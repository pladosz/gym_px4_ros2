import logging
from gym.envs.registration import register

logger = logging.getLogger(__name__)


register(
    id='px4-landing-v1',
    entry_point='gym_gazebo2_px4.envs.px4_env:SinglePx4UavEnv'
)