# check pipline locally
```
amin@amin:~/SurenaHandMultiLayer$ docker run -it --rm   -v $(pwd):$(pwd)   -v /var/run/docker.sock:/var/run/docker.sock   -w $(pwd)   efrecon/act:v0.2.24 -j build-job
```
