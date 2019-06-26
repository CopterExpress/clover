#!/usr/bin/env python3

# Generate and upload changelog

from git import Repo, exc
from github import Github
import os
import sys

upload_changelog = True

try:
    current_tag = os.environ['TRAVIS_TAG']
    if current_tag == '':
        current_tag = 'HEAD'
        upload_changelog = False
    print('TRAVIS_TAG is set to {}'.format(current_tag))
except KeyError:
    print('TRAVIS_TAG not set - not uploading changelog')
    current_tag = 'HEAD'
    upload_changelog = False

try:
    api_key = os.environ['GITHUB_OAUTH_TOKEN']
except KeyError:
    print('GITHUB_OAUTH_TOKEN not set - not uploading changelog')
    api_key = None
    upload_changelog = False

try:
    repo_slug = os.environ['TRAVIS_REPO_SLUG']
except KeyError:
    print('TRAVIS_REPO_SLUG not set - cannot determine remote repository')
    repo_slug = ''
    if upload_changelog:
        exit(1)

if len(sys.argv) > 1:
    repo_path = sys.argv[1]
else:
    repo_path = '.'

print('Opening repository at {}'.format(repo_path))
repo = Repo(repo_path)
git = repo.git()
try:
    print('Unshallowing repository')
    git.fetch('--unshallow', '--tags')
except exc.GitCommandError:
    print('Repository already unshallowed')
print('Attempting to get previous tag')
try:
    base_tag = git.describe('--tags', '--abbrev=0', '{}^'.format(current_tag))
except exc.GitCommandError:
    print('No tags found')
    base_tag = 'empty'
print('Base tag set to {}'.format(base_tag))

if base_tag == 'empty':
    changelog = git.log('--pretty=format:* %H %s *(%an)*')
else:
    changelog = git.log('{}...{}'.format(base_tag, current_tag), '--pretty=format:* %H %s *(%an)*')
print('Current changelog: \n{}'.format(changelog))

# Only interact with Github if uploading is enabled
if upload_changelog:
    gh = Github(api_key)
    gh_repo = gh.get_repo(repo_slug)
    # Get all releases and find ours by its tag name
    gh_release = None
    for release in gh_repo.get_releases():
        if release.tag_name == current_tag:
            gh_release = release
    if gh_release is None:
        # We could not find the correct release, so here's our last resort. It will most likely fail.
        gh_release = gh_repo.get_release(current_tag)
    gh_body = gh_release.body
    if gh_body is None:
        gh_body = ''
    gh_body = '{}\nChanges between `{}` and `{}`:\n\n{}'.format(gh_body, base_tag, current_tag, changelog)
    print('New release body: {}'.format(gh_body))
    gh_release.update_release(gh_release.tag_name, gh_body, draft=True, prerelease=True,
                              tag_name=gh_release.tag_name, target_commitish=gh_release.target_commitish)
