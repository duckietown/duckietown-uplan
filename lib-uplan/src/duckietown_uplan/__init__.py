# coding=utf-8 
__version__ = '1.0.0'

import logging

logging.basicConfig()
logger = logging.getLogger('dt-uplan')
logger.setLevel(logging.DEBUG)

logger.info('duckietown_uplan %s' % __version__)

from .algo import *
from .graph_utils import *
from .simulation import *
from .environment import *