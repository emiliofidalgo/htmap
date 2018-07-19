prior_file = '/home/emilio/Dropbox/Research/Code/binmap/results/binmap_prior.txt';
likelihood_file = '/home/emilio/Dropbox/Research/Code/binmap/results/binmap_likelihood.txt';
posterior_file = '/home/emilio/Dropbox/Research/Code/binmap/results/binmap_posterior.txt';

prior = load(prior_file);
likelihood = load(likelihood_file);
posterior = load(posterior_file);

[row, col] = size(prior);
figure(1);
for i=1:row
    i;
    subplot(3, 1, 1);
    plot(prior(i,:));
    title('Prior');
    subplot(3, 1, 2);
    plot(likelihood(i,:));
    title('Likelihood');
    subplot(3, 1, 3);
    plot(posterior(i,:));
    title('Posterior PDF');
    pause;
end
