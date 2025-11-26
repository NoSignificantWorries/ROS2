### On wayland

Install and start `labwc`:
```bash
DISPLAY=:1 labwc
```

Go to the task dir and run following:
```bash
cd task1
docker-compose build
docker-compose run --rm workspace tmux
# or
docker-compose run --rm workspace bash
```

