# defense

Red drones fly toward a protected asset. Blue radars track them, blue
launchers fire interceptors, and the run ends when red is defeated or the
asset is destroyed. It is the workload the framework's performance is
measured on: the store layouts, indexes and practices in
[framework/Design.md](../../framework/Design.md) were each chosen by
measuring it, idle and under contention, at up to 100,000 drones. The
details are in [Design.md](Design.md).

```sh
bazel run //application/defense -- 7            # run seed 7 headless
bazel run //application/defense:viewer -- 7     # watch it
bazel run -c opt //application/defense:defense_benchmark
bazel test //application/defense/...
```
