# Introduction
Welcome to the CSE490R blog! This repository contains a static website template built with [Hugo](https://gohugo.io/). Use it to document your successes, failures, and lessons learned as you explore the foundations of robotics! For more information on Hugo and how to install and use it, check out their [documentation](https://gohugo.io/documentation/). The section on Markdown and Hugo shortcodes may be especially useful. Note: you only need to install Hugo if you wish to build and host your blogsite locally (see below).

# Getting Started

## The Basics
1. Fork this repo to your CSE490R CSE GitLab group.
2. Open up the `config.toml` in the root directory of this repo and make the following changes:
    1. Change `title` to a name of your choosing (i.e. "Your Group's CSE490R Blog"). Do the same for `description`.
3. Open up `content/about.md` and fill it out with a short bio of your team members.

## Your First Post
To create a new blog post, make a copy of `content/example_post.md` (e.x. `content/hw0.md`) and fill out the frontmatter (author, date, and title) at the top. Next, fill out the body.

## Viewing Your Site

To view locally, run `hugo server` in the root directory of your repository. Hugo will build and host the website locally. You can view it in your browser at `localhost:1313`.

## Publishing

Once you are satisfied with your changes, commit all your work and push to the master branch. A GitLab Pipeline Runner will automatically build and host your website at https://cse490r.pages.cs.washington.edu/19sp/blogs/quiz. Note that the build process may take a few minutes. You can check the build status at https://gitlab.cs.washington.edu/cse490r/19sp/blogs/quiz/pipelines.
