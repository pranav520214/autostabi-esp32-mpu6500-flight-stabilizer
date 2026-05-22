# Publish to GitHub

## Option A: Upload with GitHub website

1. Create a new GitHub repository named `AUTOSTABI`.
2. Do not initialize it with a README, `.gitignore`, or license if you are uploading this prepared folder, because this package already includes a README and `.gitignore`.
3. Upload the contents of this folder.
4. Commit the upload.

## Option B: Push with Git command line

From inside the extracted `AUTOSTABI` folder:

```bash
git init
git add .
git commit -m "Initial AUTOSTABI release"
git branch -M main
git remote add origin https://github.com/YOUR-USERNAME/AUTOSTABI.git
git push -u origin main
```

Replace `YOUR-USERNAME` with your GitHub username.

## Before making the repo public

- Change the default Wi-Fi password in `firmware/Firmware/Firmware.ino`.
- Add a license if you want others to reuse the code.
- Add photos, wiring diagrams, and test results if available.
- Bench-test the firmware carefully before any real-world operation.
