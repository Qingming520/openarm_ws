"""Clock correction for observation freshness; wire timestamps remain unchanged."""
import math


def clock_offset(metadata, started_at, finished_at):
    values = dict(metadata or ())
    if 'lerobot-server-time' not in values:
        return 0.0
    value = float(values['lerobot-server-time']) - (started_at + finished_at) / 2
    if not math.isfinite(value):
        raise ValueError('Non-finite policy server clock offset')
    return value


def observation_age(timestamp, received_at, metadata):
    offset = float(dict(metadata or ()).get('lerobot-clock-offset', '0'))
    if not math.isfinite(offset) or not math.isfinite(timestamp):
        raise ValueError('Non-finite observation timing')
    return received_at - (timestamp + offset)
