"""Each bidder should know its initial position and set of capabilities (we assume that all robots are the same, have
the same speed, and can perform all tasks).

Upon receiving the list of tasks in the auction, the bidder tries to insert each task into its current schedule and
computes the corresponding total schedule duration (makespan). It then selects the task that would result in the
minimum makespan and submits it as a bid (task-makespan pair)."""