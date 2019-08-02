import os
import rospkg

from pygit2 import Repository, GitError

from march_launch.Color import Color
from SoftwareCheck import SoftwareCheck


class GitBranchCheck(SoftwareCheck):

    def __init__(self):
        SoftwareCheck.__init__(self, "GitBranch", "Please confirm the branches below", 1, True)

    def perform(self):
        march_launch_path = rospkg.RosPack().get_path("march_launch")
        head = os.path.split(march_launch_path)[0]
        source_path = os.path.split(head)[0]

        for repository_name in os.listdir(source_path):
            repository_path = os.path.join(source_path, repository_name)
            try:
                branch_name = Repository(repository_path).head.shorthand
                print_string = repository_name + ": " + branch_name
                if branch_name == "develop":
                    self.log(print_string, Color.Debug)
                elif "training/" in branch_name:
                    self.log(print_string, Color.Debug)
                else:
                    self.log(print_string, Color.Warning)
            except GitError as e:
                pass

        self.done = True
        self.passed = True
