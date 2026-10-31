#!/usr/bin/env python3
"""The n, mean, standard deviation, minimum and maximum that a metric file reports for a
set of observations.

A library, not a runnable script. An analyser imports it with:

    sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                    '..', 'common'))
    from metric_statistics import summarize
"""
import statistics as st


def summarize(vals):
    """Return n, mean, standard deviation, minimum and maximum of the values, or None when empty."""
    if not vals:
        return None
    return {
        'n': len(vals),
        'mean': st.mean(vals),
        'stdev': st.stdev(vals) if len(vals) > 1 else 0.0,
        'min': min(vals),
        'max': max(vals),
    }
