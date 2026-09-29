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

python3 /home/yoggi/mors_mpc/Simulator/MJCF/generate_boxes.py \
  --first-box-height 0.01 \
  --last-box-height 0.12 \
  --box-count 12 \
  --center-spacing 0.9

python3 /home/yoggi/mors_mpc/Simulator/MJCF/generate_stumps.py \
  --platform-width 2.0 \
  --step-count 5 \
  --step-height 0.05 \
  --step-depth 0.3 \
  --top-platform-length 0.8 \
  --stump-zone-length 1.8 \
  --max-stump-gap 0.25 \
  --min-stump-gap 0.05 \
  --stump-height-variation 0.08 \
  --min-stump-diameter 0.08 \
  --max-stump-diameter 0.2 \
  --seed 7
