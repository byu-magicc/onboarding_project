# Submitting to the Leaderboard

Congratulations on completing the mission! Now let's get your results on the leaderboard so others can see your achievement.

## Step 1: Gather Your Deliverables

### 1.1 Record Your Performance Statistics

When your quadrotor reaches Mr. Maeser's location, a plot will automatically appear showing your flight path and statistics.

**Record these two numbers:**
- **Minimum distance from walls (meters)** – The closest you came to any wall during the flight
- **Total time (seconds)** – How long it took to reach Mr. Maeser

These metrics determine your leaderboard ranking.

### 1.2 Create a Video

Record your successful run! This can be a simple screen recording of the RViz simulation showing your quadrotor navigating through the hallway.

**Requirements:**
- Shows the quadrotor successfully reaching Mr. Maeser
- Shows the hallway visualization clearly
- Includes the final statistics plot (or just note them separately)

**Upload options:**
- YouTube (can be unlisted if you prefer)
- Google Drive with public sharing enabled
- Any other video hosting platform that provides a shareable link

## Step 2: Submit to the Leaderboard

Now you'll create a pull request to add your results to the official leaderboard.

> [!NOTE]
> **Why a separate fork?** You'll create a second fork of the main repository (separate from your solution code) to submit just your results. This keeps the main repository clean and focused on displaying leaderboard results.

### 2.1 Fork the Main Repository (Again)

[Fork the main onboarding_project repository](https://docs.github.com/en/pull-requests/collaborating-with-pull-requests/working-with-forks/fork-a-repo) to your GitHub account. This is a **different fork** from the one containing your solution code—it's just for submitting results.

### 2.2 Edit the Leaderboard Results File

1. In your new fork, open `docs/leaderboard_results.json` in a text editor
2. You'll see a JSON array with entries like this:

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

3. **Add your entry** to the array with your information:
   - `name` – Your name or username
   - `minimum_dist_meters` – Your minimum distance from walls statistic
   - `time_s` – Your completion time
   - `github` – Link to your forked repository **containing your solution code**
   - `video` – Link to your video

> [!IMPORTANT]
> **JSON syntax is picky!** Pay attention to:
> - Commas between entries (but not after the last entry)
> - Matching curly braces `{}` and brackets `[]`
> - Quotes around strings
> - No trailing commas
> 
> If your JSON syntax is invalid, the automatic leaderboard update will fail and your PR won't be merged.

### 2.3 Commit and Push Your Changes

Commit the changes to `docs/leaderboard_results.json` and push them to the **main branch** of your leaderboard fork (**not** your solution code repository).

### 2.4 Create a Pull Request

1. Go to [the main repository's Pull Requests page](https://github.com/byu-magicc/onboarding_project/pulls)
2. Click **"New Pull Request"**
3. Click the **"compare across forks"** link at the top
4. In the **"head repository"** dropdown, select your leaderboard fork (not your solution code fork)
5. Ensure you're comparing `main` branch to `main` branch
6. Click **"Create Pull Request"**
7. Add a title and description (e.g., "Add [Your Name] to leaderboard")
8. Submit the pull request

### 2.5 Wait for Review

Lab members will review your PR to verify:
- Your video shows successful completion
- The JSON syntax is correct
- Your statistics match what's shown in your video

If everything looks good, we'll merge it and your name will appear on the leaderboard! If there are issues, we'll comment on your PR with requested changes.

---

## Submitting Multiple Times

You're welcome to improve your solution and resubmit.

**How to handle multiple submissions:**

**Option 1: Add a new entry** – Each submission gets its own entry in the JSON file. The leaderboard will automatically display only your best score, but all your attempts will be recorded.

**Option 2: Update your existing entry** – Replace your previous statistics with your new ones. This keeps the file cleaner.

Notice in the example above that Cosmo has two entries with different scores. On the actual leaderboard, only the best score appears with his name.

> [!TIP]
> Keep improving! The leaderboard rewards both safety (staying far from walls) and speed (fast completion time). Remember you are free to use ROScopter however your want.
