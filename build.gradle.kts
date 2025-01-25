plugins {
    `java-library`
    alias(libs.plugins.wpilib.repos)
}

repositories {
    mavenCentral()
}

wpilibRepositories.addAllReleaseRepositories(project)

dependencies {
    implementation("edu.wpi.first.cscore:cscore-java:2025.2.1")
    implementation("edu.wpi.first.wpilibj:wpilibj-java:2025.2.1")
    implementation("edu.wpi.first.wpiutil:wpiutil-java:2025.2.1")
    implementation("edu.wpi.first.wpimath:wpimath-java:2025.2.1")
    implementation("edu.wpi.first.hal:hal-java:2025.2.1")

    testImplementation(libs.junit.jupiter)

    testRuntimeOnly("org.junit.platform:junit-platform-launcher")
}

java {
    toolchain {
        languageVersion = JavaLanguageVersion.of(17)
    }
}

tasks.named<Test>("test") {
    useJUnitPlatform()
}

tasks.register("listrepos") {
    val repos = project.repositories;
    doLast {
        println("Repositories:")
        repos.map{it as MavenArtifactRepository}
            .forEach{
            println("Name: ${it.name}; url: ${it.url}")
        }
    }
}