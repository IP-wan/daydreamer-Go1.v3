import pathlib
import sys
import warnings

# Отключает предупреждения о возможных ошибках или неправильном использовании функции
warnings.filterwarnings('ignore', '.*box bound precision lowered.*')
warnings.filterwarnings('ignore', '.*using stateful random seeds*')
warnings.filterwarnings('ignore', '.*is a deprecated alias for.*')

# Присваиваем переменной путь от домашней папки пользователя до текущего файла
directory = pathlib.Path(__file__)

try:
    import google3  # noqa
except ImportError:
    directory = directory.resolve()
# Присваиваем переменной путь к родительскому каталогу файла.
directory = directory.parent
# Добавляем пути в модуль sys, для поиска модулей Python.
sys.path.append(str(directory.parent))
sys.path.append(str(directory.parent.parent.parent))
# Присваиваем переменной название родительской директории.
__package__ = directory.name

import embodied


def main(argv=None):
    from . import agent as agnt

    # Присваивает переменной parsed словарь, состоящий из именованных аргументов.
    # Присваивает переменной other список, состоящий из аргументов командной строки.
    parsed, other = embodied.Flags(
        configs=['defaults'], worker=0, workers=1,
        learner_addr='localhost:2222',
    ).parse_known(argv)
    # Присваиваем переменной config данные из файла embodied/agents/dreamerv2plus/configs.yaml.
    config = embodied.Config(agnt.Agent.configs['defaults'])
    for name in parsed.configs:
        # Обновляем данные конфигурации названием робота
        config = config.update(agnt.Agent.configs[name])
    # Объединяем аргументы командной строки с конфигурационными данными
    config = embodied.Flags(config).parse(other)
    # Форматируем путь к папке logdir
    config = config.update(logdir=str(embodied.Path(config.logdir)))
    # Присваиваем переменной args путь к директории logdir и конфигурационные данные для тренировки
    args = embodied.Config(logdir=config.logdir, **config.train)
    args = args.update(expl_until=args.expl_until // config.env.repeat)
    print(config)

    # Присваиваем переменной logdir путь из экземпляра config
    logdir = embodied.Path(config.logdir)
    # Присваиваем переменной step счетчик взаимодействий сл средой
    step = embodied.Counter()

    outdir = logdir
    # Присваиваем переменной multiplier множитель (количество повторений)
    multiplier = config.env.repeat
    if config.run == 'acting':
        outdir /= f'worker{parsed.worker}'
        multiplier *= parsed.workers
    elif config.run == 'learning':
        outdir /= 'learner'
        multiplier = 1
    # Выводит лог-информацию в консоль и в файлы
    logger = embodied.Logger(step, [
        embodied.logger.TerminalOutput(config.filter),
        embodied.logger.JSONLOutput(outdir, 'metrics.jsonl'),
        embodied.logger.JSONLOutput(outdir, 'scores.jsonl', 'episode/score'),
        embodied.logger.TensorBoardOutput(outdir),
    ], multiplier)

    cleanup = []
    try:
        # Задаем начальное значение (не меняется от перезапуска)
        config = config.update({'env.seed': hash((config.seed, parsed.worker))})
        env = embodied.envs.load_env(
            config.task, mode='train', logdir=logdir, **config.env)
        agent = agnt.Agent(env.obs_space, env.act_space, step, config)
        cleanup.append(env)

        if config.run == 'train':
            replay = make_replay(config, logdir / 'episodes')
            embodied.run.train(agent, env, replay, logger, args)

        elif config.run == 'train_eval':
            eval_env = embodied.envs.load_env(
                config.task, mode='eval', logdir=logdir, **config.env)
            replay = make_replay(config, logdir / 'episodes')
            eval_replay = make_replay(config, logdir / 'eval_episodes', is_eval=True)
            embodied.run.train_eval(
                agent, env, eval_env, replay, eval_replay, logger, args)
            cleanup.append(eval_env)

        elif config.run == 'train_fixed_eval':
            if config.eval_dir:
                assert not config.train.eval_fill
                eval_replay = make_replay(config, config.eval_dir, is_eval=True)
            else:
                assert config.train.eval_fill
                eval_replay = make_replay(
                    config, logdir / 'eval_episodes', is_eval=True)
            replay = make_replay(config, logdir / 'episodes')
            embodied.run.train_fixed_eval(
                agent, env, replay, eval_replay, logger, args)

        elif config.run == 'learning':
            env.close()
            port = parsed.learner_addr.split(':')[-1]
            replay = make_replay(config, logdir / 'episodes', server_port=port)
            if config.eval_dir:
                eval_replay = make_replay(config, config.eval_dir, is_eval=True)
            else:
                eval_replay = replay
            embodied.run.learning(agent, replay, eval_replay, logger, args)

        elif config.run == 'acting':
            replay = make_replay(config, remote_addr=parsed.learner_addr)
            embodied.run.acting(agent, env, replay, logger, outdir, args)

        else:
            raise NotImplementedError(config.run)
    finally:
        for obj in cleanup:
            obj.close()


def make_replay(
        config, directory=None, is_eval=False, remote_addr=None,
        server_port=None, **kwargs):
    size = config.replay_size
    chunk = config.replay_chunk
    if is_eval:
        size //= 10
    if remote_addr:
        store = embodied.replay.StoreClient(remote_addr)
    else:
        if directory:
            # Выполняется
            store = embodied.replay.CkptRAMStore(
                directory, size, parallel=True)
        else:
            store = embodied.replay.RAMStore(size)
    store = embodied.replay.Stats(store)
    if server_port:
        store = embodied.replay.StoreServer(store, server_port)

    if config.replay == 'fixed':
        replay = embodied.replay.FixedLength(
            store, chunk, **config.replay_fixed, **kwargs)
    elif config.replay == 'consec':
        replay = embodied.replay.Consecutive(
            store, chunk, **config.replay_consec, **kwargs)
    elif config.replay == 'prio' and not is_eval:
        replay = embodied.replay.Prioritized(
            store, chunk, **config.replay_prio, **kwargs)
    elif config.replay == 'prio' and is_eval:
        # Don't use prioritized for eval.
        replay = embodied.replay.FixedLength(
            store, chunk, **config.replay_fixed, **kwargs)
    else:
        raise NotImplementedError(config.replay)
    return replay


if __name__ == '__main__':
    main()
