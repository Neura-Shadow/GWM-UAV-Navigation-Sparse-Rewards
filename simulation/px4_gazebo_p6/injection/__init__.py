"""P6 fault injection (M2): declared, observable fault injectors around the accepted P5 stack.

Nothing here edits a P2–P5 file. Every injector has three modes: `off` (not launched), `pass-through`
(present, changes nothing) and `active` (applies exactly the declared fault). Every activation and every
affected item is written to the run's fault ledger, which the evaluator never trusts on its own.
"""
