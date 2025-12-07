# Deployment Guide

## ✅ GitHub Repository Setup - COMPLETED

Your code is now on GitHub: https://github.com/codewithurooj/hackathon_book

## 🌐 GitHub Pages Deployment - IN PROGRESS

### Automatic Setup (Recommended)

The GitHub Actions workflow has been configured and will automatically deploy your website when you push to `main` branch.

### Manual Steps Required:

1. **Go to your repository**: https://github.com/codewithurooj/hackathon_book
2. **Click "Settings"** (top menu)
3. **Click "Pages"** (left sidebar)
4. **Under "Source"**:
   - Select: **GitHub Actions** (not "Deploy from a branch")
5. **Wait 2-3 minutes** for the first deployment
6. **Your site will be live at**: https://codewithurooj.github.io/hackathon_book/

### Troubleshooting GitHub Pages

If deployment fails:
1. Go to **Actions** tab in your repo
2. Click on the failed workflow
3. Check the error logs
4. Common issues:
   - Node.js version mismatch → Fixed in workflow (using Node 18)
   - Build errors → Check build locally first

## 🚂 Railway Backend Deployment

### Prerequisites

1. **Create Railway Account**: https://railway.app
2. **Install Railway CLI** (optional):
   ```bash
   npm install -g @railway/cli
   ```

### Method 1: Railway Dashboard (Easiest)

#### Step 1: Create New Project

1. Go to https://railway.app/dashboard
2. Click **"New Project"**
3. Select **"Deploy from GitHub repo"**
4. Select your repository: `codewithurooj/hackathon_book`
5. Railway will detect your project

#### Step 2: Configure Backend Service

1. Click **"Add variables"** or **"Variables"** tab
2. Add the following environment variables:

```env
# OpenAI Configuration
OPENAI_API_KEY=your-openai-api-key-here

# Qdrant Configuration
QDRANT_URL=your-qdrant-url-here
QDRANT_API_KEY=your-qdrant-api-key-here
QDRANT_COLLECTION_NAME=physical-ai-book

# FastAPI Configuration
API_BASE_URL=${{RAILWAY_STATIC_URL}}
CORS_ORIGINS=["https://codewithurooj.github.io"]
BACKEND_API_KEY=your-backend-api-key-here

# PostgreSQL Configuration (use your Neon connection string)
DATABASE_URL=your-neon-database-url-here

# Environment
ENVIRONMENT=production
LOG_LEVEL=info
EMBEDDING_MODEL=text-embedding-3-small
CHAT_MODEL=gpt-4o
```

#### Step 3: Configure Build Settings

Railway will auto-detect Python, but let's be explicit:

1. **Root Directory**: `backend`
2. **Build Command**: `pip install -r requirements.txt`
3. **Start Command**: `uvicorn app.main:app --host 0.0.0.0 --port $PORT`

#### Step 4: Create railway.json (Optional)

Create this file in `backend/railway.json`:

```json
{
  "$schema": "https://railway.app/railway.schema.json",
  "build": {
    "builder": "NIXPACKS",
    "buildCommand": "pip install -r requirements.txt"
  },
  "deploy": {
    "startCommand": "uvicorn app.main:app --host 0.0.0.0 --port $PORT",
    "restartPolicyType": "ON_FAILURE",
    "restartPolicyMaxRetries": 10
  }
}
```

#### Step 5: Deploy

1. Click **"Deploy"** button
2. Railway will:
   - Clone your repo
   - Install dependencies
   - Run migrations (you may need to do this manually)
   - Start the server
3. You'll get a URL like: `https://your-app.railway.app`

#### Step 6: Run Database Migration

Since Railway doesn't run migrations automatically, you have two options:

**Option A: Using Railway CLI**
```bash
railway login
railway link
railway run python backend/run_migration.py
```

**Option B: Using Neon SQL Editor**
- Already done! ✅ Tables are created in your Neon database

### Method 2: Railway CLI

```bash
# Login to Railway
railway login

# Create new project
railway init

# Link to GitHub repo (optional)
railway link

# Add environment variables
railway variables set OPENAI_API_KEY="your-key"
railway variables set DATABASE_URL="your-neon-url"
# ... add all other variables

# Deploy
railway up
```

## 📝 Post-Deployment Steps

### 1. Update Frontend API URL

After Railway deployment, update your frontend to use the Railway backend URL:

1. Create `website/.env.production`:
```env
REACT_APP_API_URL=https://your-app.railway.app/api/v1
```

2. Commit and push:
```bash
git add website/.env.production
git commit -m "Add production API URL"
git push origin main
```

### 2. Update CORS Settings

Update your Railway environment variable:
```env
CORS_ORIGINS=["https://codewithurooj.github.io"]
```

### 3. Test the Deployment

1. **Frontend**: Visit https://codewithurooj.github.io/hackathon_book/
2. **Backend**: Visit https://your-app.railway.app/docs
3. **Translation Feature**:
   - Open browser console
   - Run: `localStorage.setItem('user_id', '123')`
   - Navigate to any chapter
   - Click "Add Urdu Translation" button
   - Submit a translation
   - Check it works!

## 🔧 Troubleshooting

### GitHub Pages Issues

**Site not loading**:
- Check Actions tab for build status
- Verify baseUrl in docusaurus.config.ts is `/hackathon_book/`
- Check GitHub Pages settings (Settings > Pages)

**Styles/assets not loading**:
- Ensure baseUrl matches your repo name
- Check browser console for 404 errors

### Railway Issues

**Build fails**:
- Check build logs in Railway dashboard
- Verify all environment variables are set
- Check requirements.txt includes all dependencies

**Database connection fails**:
- Verify DATABASE_URL is correct
- Check Neon database is active
- Test connection locally first

**API not responding**:
- Check Railway logs
- Verify PORT is bound correctly: `--port $PORT`
- Test endpoints at `/docs`

## 📊 Monitoring

### GitHub Pages
- Check Actions tab for deployment status
- Monitor at: https://github.com/codewithurooj/hackathon_book/actions

### Railway
- Dashboard: https://railway.app/dashboard
- Logs: Click on your service → Logs tab
- Metrics: CPU, Memory, Network usage

## 🎉 Success!

Once deployed, your site will be accessible at:

- **Frontend**: https://codewithurooj.github.io/hackathon_book/
- **Backend**: https://your-app.railway.app
- **API Docs**: https://your-app.railway.app/docs

## Next Steps

1. ✅ Test all features on production
2. ✅ Monitor logs for errors
3. ✅ Set up custom domain (optional)
4. ✅ Enable HTTPS (GitHub Pages does this automatically)
5. ✅ Set up error tracking (Sentry, etc.)

---

**Need Help?**
- GitHub Pages: https://docs.github.com/en/pages
- Railway: https://docs.railway.app
- Docusaurus: https://docusaurus.io/docs/deployment
