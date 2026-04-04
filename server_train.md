# For training:
### one GPU
    python examples/ur10_gello/ur10_train.py \
      --dataset-repo-id /home_local/rudra_1/guningquan/dataset/testlerobot \
      --policy-type act \
      --output-dir /home_local/rudra_1/guningquan/checkpoints \
      --job-name act_test \
      --policy-device cuda \
      --wandb-enable false \
      --policy-repo-id guningquan/act_policy
  
### MULTI-GPU
    CUDA_VISIBLE_DEVICES=0,1 accelerate launch --multi_gpu --num_processes=2 \
      examples/ur10_gello/ur10_train.py \
      --dataset-repo-id /home_local/rudra_1/guningquan/dataset/testlerobot \
      --policy-type act \
      --output-dir /home_local/rudra_1/guningquan/checkpoints  \
      --job-name act_test \
      --policy-device cuda \
      --wandb-enable false \
      --policy-repo-id guningquan/act_policy

# for inference

### a. The remote server listens and waits for computation (on server pc)：
    python examples/ur10_gello/ur10_train.py --serve-policy \
      --policy-path /home_local/rudra_1/guningquan/checkpoints/checkpoints/000200  \
      --serve-dataset-from-train-config

### b. Local control of robot and camera operation (on robot pc):
    python examples/ur10_gello/client_ur10_control.py \
      --server-host 10.245.91.19 --server-port 8765 \
      --control-dt 0.033 \
      --max-steps 300 \
      --num-rollouts 2  \
      --video-dir /path/to/videos \
      --reset-between-rollouts \
      --reset-policy


