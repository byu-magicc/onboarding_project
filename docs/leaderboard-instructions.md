# Submitting to the leaderboard

After successfully completing [the mission](/docs/project-instructions.md), you can submit your scores to the leaderboard!
To do this, follow these instructions:

## Gather Deliverables

1. **Collect your scores:**
When arriving at Mr. Maeser's location, a plot of the minimum distance vs time will appear.
Note the reported minimum distance and the total time (the time it took the drone to reach the end of the hallway).

2. **Create a Video:**
Please create a video of your solution.
This can be a simple screen recording you take of the RViz simulation.
Upload the video to YouTube.

## Submit

1. To submit your deliverables, first [fork](https://docs.github.com/en/pull-requests/collaborating-with-pull-requests/working-with-forks/fork-a-repo) this repository (again).
The fork you create on this step **should not** contain your solution code.
We will be editing only one file before submitting a [pull request](https://docs.github.com/en/pull-requests/collaborating-with-pull-requests/proposing-changes-to-your-work-with-pull-requests/about-pull-requests) (PR) to merge your scores into the main repository.

2. Open the `docs/leaderboard_results.json` file in a text editor.
You will see something like

    ```json
    [
      {
        "name": "Cosmo",
        "minimum_dist_meters": 2.0,
        "time_s": 177.2,
        "github": "https://github.com/byu-magicc/onboarding_project",
        "video": "https://youtube.com"
      },
      {
        "name": "Cosmo",
        "minimum_dist_meters": 0.01,
        "time_s": 64.3,
        "github": "https://github.com/byu-magicc/onboarding_project",
        "video": "https://youtube.com"
      }
    ]
    ```

3. Add your submission details by adding your name, minimum distance and total time statistics, a link to your GitHub link containing your solution code, and a link to your YouTube video.

    You can copy and past the second entry and edit from there. Be careful about the commas! If the syntax is not correct, your PR will fail.

4. Commit and push those changes to the `docs/leaderboard_results.json` file to the **main branch** of your newly forked repo (**not the repo containing your solution code**).

5. On [this repository on GitHub.com](https://github.com/byu-magicc/onboarding_project/pulls), navigate to the "Pull Request" tab.

6. Click "New Pull Request".

7. At the top of this new screen, you will see a link saying "compare across forks". Click that link.

8. You will see a box starting with "**head repository: ...**". Click there and find your fork (**not the one with your solution code**). You should be comparing your main branch with our main branch.

9. Select "Create Pull Request", add some details, and submit the PR.

10. We will then review the PR and merge it in if it all looks good! If something is wrong, we'll ask you to make revisions.

### Submitting multiple times to the leaderboard
You will notice Cosmo submitted twice to the leaderboard, but his name (and best score) will only appear once on the leaderboard.

You are free to make as many changes and submissions as you want.
When adding additional submissions, you can also choose to add a new entry for each submission, or you can overwrite your previous submission.
