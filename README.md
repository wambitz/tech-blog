# Jekyll Blog on GitHub pages

## Create your Docker image

This will allow you to install Jekyll locally for testing and not have to worry Ruby dependencies.

```bash
docker build -t jekyll-blog .
```

## Create your GitHub repository


## Create a New Jekyll Site

### Host Terminal

This will create the `jekyll-blog` default content in the local `host` workspace and install all the template files for later to be mounted in `/srv/jekyll`.


```bash
docker run --rm --name jekyll-blog -v ${PWD}:/srv/jekyll jekyll-blog jekyll new . 

# Use "--force" if you want to overwrite existent content, i.e.
# docker run --rm --name jekyll-blog -v ${PWD}:/srv/jekyll jekyll-blog jekyll new . --force
```

### Devcontainer

If you desire to do this manually inside the container it is possible to use the `devcontainer` to achieve the same result.

After creating your image use `F1` or `Shift + Ctrl + P` and type `Open in devcontainer`.

Once you have a terminal open type:

```bash
jekyll new .

# Same as mention in the previous section use "--force" if content was created previously.
# jekyll new . --force
```

## Test locally with the Jekyll Server

### Host Terminal

Run in development mode, this will reload your page as you make changes. 

**NOTE:** Installing dependencies separately i.e., running `bundle install` and then `jekyll serve` will not work.

Both instructions need to be run together, otherwise jekyll cannot find the gems.

```bash
docker run --rm --name jekyll-blog -v ${PWD}:/srv/jekyll -p 4000:4000 jekyll-blog bash -c "bundle install && bundle exec jekyll serve --host 0.0.0.0 --livereload"
```

Open your web browser and paste the URL: `http://localhost:4000/jekyll-blog`

### Docker compose

For simplicity `docker-compose.yml` contains this configuration:

```bash
docker-compose up -d
```

To remove and stop the containers:

```bash
docker-compose down
```

Open your web browser and paste the URL: `http://localhost:4000/jekyll-blog`

### Devcontainer

```bash
bundle install
bundle exec jekyll serve --host 0.0.0.0 --livereload
```

**Important**: Use `http://localhost:4001/jekyll-blog` instead of `4000`


## Create GitHub Page

Follow the GitHub [official documentation](https://docs.github.com/en/pages/setting-up-a-github-pages-site-with-jekyll/creating-a-github-pages-site-with-jekyll).

## Known Issues:

It looks like the gems are correctly installed in the Docker container once `bundle install` is manually run within an interactive session. However, when running the jekyll serve command directly via `docker run`, the container isn not finding the gems that were supposedly installed.

**Understanding the Problem**: The issue is likely due to the fact that when `bundle install` is run in an interactive session, the gems are installed in the container's filesystem. However, when `jekyll serve` is run in a non-interactive Docker command, it's possible that the gems aren't being recognized because they weren't installed in the current session or due to some caching or volume mapping issues.

