@Library('bitbots_jenkins_library') import de.bitbots.jenkins.*;

defineProperties()

def pipeline = new BitbotsPipeline(this, env, currentBuild, scm)
pipeline.configurePipelineForPackage(new PackagePipelineSettings(new PackageDefinition("white_balancer")))
pipeline.execute()