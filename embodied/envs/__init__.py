import functools

import embodied

from .go1 import Go1


def load_env(
    task, amount=1, parallel='none', daemon=False, restart=False, seed=None,
    kbreset=False, **kwargs):
  ctors = []
  for index in range(amount):
    ctor = functools.partial(load_single_env, task, **kwargs)
    if seed is not None:
      ctor = functools.partial(ctor, seed=hash((seed, index)) % (2 ** 31 - 1))
    if parallel != 'none':
      ctor = functools.partial(embodied.Parallel, ctor, parallel, daemon)
    if restart:
      ctor = functools.partial(embodied.wrappers.RestartOnException, ctor)
    if kbreset:
      from .kbreset import KBReset
      ctor = functools.partial(KBReset, ctor)
    ctors.append(ctor)
  envs = [ctor() for ctor in ctors]
  return embodied.BatchEnv(envs, parallel=(parallel != 'none'))


def load_single_env(
    task, size=(64, 64), repeat=1, mode='train', camera=-1, gray=False,
    length=0, logdir='/dev/null', discretize=0, sticky=True, lives=False,
    episodic=True, resets=True, seed=None):
  suite, task = task.split('_', 1)
  if suite == 'go1':
    assert size == (64, 64), size
    env = Go1(task, repeat, length or 1000, True)
  else:
    raise NotImplementedError(suite)
  for name, space in env.act_space.items():
    if name == 'reset':
      continue
    if space.discrete:
      env = embodied.wrappers.OneHotAction(env, name)
    elif discretize:
      env = embodied.wrappers.DiscretizeAction(env, name, discretize)
    else:
      env = embodied.wrappers.NormalizeAction(env, name)
  if length:
    env = embodied.wrappers.TimeLimit(env, length, resets)
  return env


__all__ = [
    k for k, v in list(locals().items())
    if type(v).__name__ in ('type', 'function') and not k.startswith('_')]
