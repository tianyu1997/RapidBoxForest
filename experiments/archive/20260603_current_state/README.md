# Current Experiment Section Archive

This directory records the state before the TRO 2026 experiment redesign.

The current manuscript experiment section was extracted to:

`paper/history/experiments_section_20260603.tex`

Known issues in the archived state:

1. `paper/sbf_tro_2026.tex` imports experiment tables and figures from
   `paper/sbf_old/generated/`.
2. Several current experiment dispatchers call scripts under
   `safe_box_forest/experiments/sbf_old/`.
3. Random-scene generation and replay are only partially cataloged.
4. Exp.7 is a current dynamic-update experiment and must be kept in the new
   active experiment plan.

