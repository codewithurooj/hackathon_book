# CI/CD Pipeline Definition for Docusaurus Frontend

This document outlines a high-level CI/CD pipeline for the Docusaurus frontend application.

## Goals

*   Automate building, testing, and deployment of the frontend.
*   Ensure code quality and consistency.
*   Enable rapid and reliable delivery of new content and features.
*   Support deployment to static hosting services (e.g., GitHub Pages, Netlify, Vercel).

## Proposed Tools

*   **Version Control**: Git (GitHub, GitLab, Bitbucket)
*   **CI/CD Platform**: GitHub Actions, GitLab CI, Jenkins, CircleCI
*   **Static Hosting**: GitHub Pages, Netlify, Vercel, AWS S3 + CloudFront

## Pipeline Stages (Example: GitHub Actions)

### 1. Build & Test (on push to `develop` or feature branches)

*   **Trigger**: `on: [push]` to `branches: [develop, 'feature/**']`
*   **Jobs**:
    *   **Install Dependencies**:
        *   Navigate to `website/` directory.
        *   Run `npm install`.
    *   **Linting & Formatting**:
        *   Run `npm run lint`.
        *   Run `npm run format --check`.
    *   **Unit Tests**:
        *   Run `npm test`.
    *   **Build Docusaurus Site**:
        *   Run `npm run build`.
        *   Artifacts generated in `website/build/`.

### 2. Deploy to Staging (on successful merge to `develop`)

*   **Trigger**: `on: [push]` to `branches: [develop]`
*   **Dependencies**: Requires "Build & Test" job to pass.
*   **Jobs**:
    *   **Deploy**:
        *   Deploy `website/build/` artifacts to a staging environment (e.g., Netlify Preview, dedicated S3 bucket).
        *   Perform basic health checks.

### 3. Release & Deploy to Production (on tag push or manual approval)

*   **Trigger**: `on: [push]` to `tags: ['v*.*.*']` or manual workflow dispatch.
*   **Dependencies**: Requires "Deploy to Staging" to pass and (optional) manual approval.
*   **Jobs**:
    *   **Deploy**:
        *   Deploy `website/build/` artifacts to production static hosting (e.g., GitHub Pages, Netlify Production, Vercel Production).
        *   Invalidate CDN cache (if applicable).
        *   Perform comprehensive health checks.
    *   **Rollback Strategy**: Define clear rollback procedures for static sites (e.g., redeploy previous successful build).

## Environment Variables (Frontend)

Sensitive frontend API keys (e.g., `REACT_APP_CHATBOT_API_KEY`) must be exposed via build-time environment variables, ensuring they are not committed to the repository. Note that these are client-side variables and are publicly accessible after deployment.
