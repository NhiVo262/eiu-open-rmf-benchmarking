#!/usr/bin/env python3

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
