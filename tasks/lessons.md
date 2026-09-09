# Lessons

- PWM actions must retain their actual timestamps. A shorter fixed tick only reduces quantisation; advance the old state to each transition before applying the new state.
- Distinguish exact command-duration accounting from numerical thermal integration accuracy when describing guarantees.
