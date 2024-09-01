# Use the official Ruby image as the base image
FROM ruby:3.2

# Set an environment variable to avoid installation prompts during gem installation
ENV DEBIAN_FRONTEND=noninteractive

# Install dependencies
RUN apt-get update && \
    apt-get install -y \
    build-essential \
    nodejs && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Install Jekyll and Bundler
RUN gem install jekyll bundler

# Create a directory for the Jekyll site
WORKDIR /srv/jekyll

# Expose the default Jekyll port
EXPOSE 4000

# By default, serve the Jekyll site
CMD ["/bin/bash"]
