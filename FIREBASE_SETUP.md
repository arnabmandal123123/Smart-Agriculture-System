# Firebase Setup Guide for Portfolio

Follow these steps to set up Google Firebase for your portfolio's visit counter and like functionality.

## Step 1: Create a Firebase Project

1. Go to [Firebase Console](https://console.firebase.google.com/)
2. Click "Add project" or "Create a project"
3. Enter project name: `portfolio-stats` (or any name you prefer)
4. Click "Continue"
5. Disable Google Analytics (optional, not needed for this project)
6. Click "Create project"
7. Wait for the project to be created, then click "Continue"

## Step 2: Register Your Web App

1. In the Firebase Console, click the **Web icon** (`</>`) to add a web app
2. Enter app nickname: `Portfolio Website`
3. **Check** "Also set up Firebase Hosting" (optional)
4. Click "Register app"
5. You'll see your Firebase configuration code - **COPY THIS!**

## Step 3: Enable Realtime Database

1. In the left sidebar, click **"Build"** → **"Realtime Database"**
2. Click **"Create Database"**
3. Select location: Choose closest to your audience (e.g., `us-central1`, `asia-southeast1`)
4. Security rules: Select **"Start in test mode"** (we'll update this)
5. Click **"Enable"**

## Step 4: Configure Database Security Rules

1. In Realtime Database, go to the **"Rules"** tab
2. Replace the default rules with this:

```json
{
  "rules": {
    "stats": {
      "visits": {
        ".read": true,
        ".write": true
      },
      "likes": {
        ".read": true,
        ".write": true
      }
    }
  }
}
```

3. Click **"Publish"**

**Note:** These rules allow anyone to read/write. For production, consider adding rate limiting or more restrictive rules.

## Step 5: Update Your Portfolio Code

1. Open `script.js` in your portfolio folder
2. Find the `CONFIG` object at the top
3. Replace the `firebase` section with your configuration:

```javascript
firebase: {
    apiKey: "AIzaSyXXXXXXXXXXXXXXXXXXXXXXXXXXXXX",
    authDomain: "your-project-id.firebaseapp.com",
    databaseURL: "https://your-project-id-default-rtdb.firebaseio.com",
    projectId: "your-project-id",
    storageBucket: "your-project-id.appspot.com",
    messagingSenderId: "123456789012",
    appId: "1:123456789012:web:abcdefghijklmnop"
}
```

**Where to find these values:**
- Go to Firebase Console → Project Settings (gear icon)
- Scroll down to "Your apps"
- Find your web app and click the config icon
- Copy the configuration object

## Step 6: Initialize Database Data (Optional)

To set initial values:

1. In Realtime Database, click the **"Data"** tab
2. Click the **+** button next to the database URL
3. Add:
   - Name: `stats`
   - Click **+** to add children:
     - Name: `visits`, Value: `0`
     - Name: `likes`, Value: `0`
4. Click **"Add"**

## Step 7: Test Your Portfolio

1. Open your `index.html` in a browser
2. You should see:
   - **Total Visits** counter incrementing
   - **Likes** counter showing current count
   - **Like button** that you can click

## Database Structure

Your Firebase Realtime Database will have this structure:

```
portfolio-stats (root)
└── stats
    ├── visits: 1234
    └── likes: 567
```

## Troubleshooting

### "Permission denied" error
- Check your database rules are set correctly
- Make sure you published the rules

### Counters show "---"
- Check if Firebase config is correct in `script.js`
- Open browser console (F12) to see error messages
- Verify `databaseURL` is correct (should end with `.firebaseio.com`)

### Visit count not incrementing
- Check browser console for errors
- Verify internet connection
- Check Firebase Console → Realtime Database → Data tab to see if values are updating

### Like button not working
- Clear browser cache and localStorage
- Check browser console for errors

## Security Best Practices (Optional - For Production)

For better security, update your database rules:

```json
{
  "rules": {
    "stats": {
      "visits": {
        ".read": true,
        ".write": "!data.exists() || newData.val() === data.val() + 1"
      },
      "likes": {
        ".read": true,
        ".write": "!data.exists() || newData.val() >= data.val() - 1 && newData.val() <= data.val() + 1"
      }
    }
  }
}
```

This ensures:
- Visits can only increment by 1
- Likes can only increment or decrement by 1

## Monitoring Usage

1. Go to Firebase Console → Realtime Database
2. Click **"Usage"** tab to see:
   - Connections
   - Storage
   - Bandwidth
   - Operations

Free tier includes:
- 1 GB storage
- 10 GB/month bandwidth
- 100 simultaneous connections

This is more than enough for a portfolio website!

## Need Help?

- Check Firebase documentation: https://firebase.google.com/docs/database
- Firebase Console: https://console.firebase.google.com/
- Check browser console (F12) for error messages

---

**Your Firebase setup is complete!** 🎉

Your portfolio will now track visits and likes in real-time!
