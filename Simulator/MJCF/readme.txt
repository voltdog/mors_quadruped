In order to generate the scene with stairs it is needed to use the command:

python3 /home/yoggi/mors_mpc/Simulator/MJCF/generate_stairs.py \
  --step-height 0.05 \
  --step-length 0.3 \
  --step-count 5 \
  --top-platform-size 1.0


In order to generate the scene with free-falling boards it is needed to use the command:

python3 /home/yoggi/mors_mpc/Simulator/MJCF/generate_boards.py \
  --board-count 12 \
  --mean-length 0.7 \
  --mean-thickness 0.05 \
  --mean-width 0.25 \
  --x-span 10.0 \
  --mixed-materials \
  --seed 7
